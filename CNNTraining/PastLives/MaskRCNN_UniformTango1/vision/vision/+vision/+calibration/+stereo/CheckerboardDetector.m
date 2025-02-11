classdef CheckerboardDetector < vision.calibration.PatternDetector
    % CheckerboardDetector Interface for checkerboard detection for stereo camera calibration.
    %
    %   CheckerboardDetector specifies the interface for a checkerboard
    %   detector to be used for stereo camera calibration in the Stereo Camera
    %   Calibrator App and from the command line.
    %
    %   For more information on defining a custom calibration pattern detector
    %   using this interface, see the <a href="matlab:help('vision.calibration.PatternDetector')">PatternDetector</a> class:
    %
    %       edit vision.calibration.PatternDetector
    %
    %   See also stereoCameraCalibrator, detectCheckerboardPoints,
    %   generateCheckerboardPoints, vision.calibration.PatternDetector.

    % Copyright 2021-2024 The MathWorks, Inc.

    %----------------------------------------------------------------------
    properties(Constant)
        % Change this value to change the name in the "Choose Pattern" drop down.
        Name = vision.getMessage('vision:caltool:DefaultPattern');
    end

    %----------------------------------------------------------------------
    properties
        WorldUnits = vision.getMessage('vision:caltool:millimeters');

        Panel
    end

    %----------------------------------------------------------------------
    % UI components for the properties panel
    %----------------------------------------------------------------------
    properties (Access = private)
        SquareSizeEditBox

        UnitsPopup

        AvailableUnits = vision.internal.calibration.availableWorldUnits;

        PropertyPanelGridLayout

        RadioButtonGroup

        LowDistortionButton

        HighDistortionButton

        MinCornerMetricEditBox

        CornerMetricModeRadioButtonGroup

        AutoCornerMetricModeButton

        ManualCornerMetricModeButton
    end

    %----------------------------------------------------------------------
    properties
        SquareSize = 25;

        BoardSize

        IsDistortionHigh = false;

        MinCornerMetric = 0.15;
    end

    methods

        %------------------------------------------------------------------
        % Detection method for pattern keypoint detection in the
        % calibration images
        %------------------------------------------------------------------
        function [imagePoints, pairsUsed] = detectPatternPoints(this, imageFileNames1, imageFileNames2, varargin)

            % Handle command line usage (Name-Value pairs are used to
            % set properties inplace of the properties panel in the calibrator app)
            parser = inputParser;
            parser.addParameter('ShowProgressBar', false, @checkShowProgressBar);
            parser.addParameter('ProgressBarParent', [], @checkProgressBarParent);

            cornerMetricDef = 0.15; % Default for low distortion.
            parser.addParameter('MinCornerMetric', cornerMetricDef, @checkMinCornerMetric);
            parser.addParameter('HighDistortion', false, @checkHighDistortion);
            parser.addParameter('PartialDetections', true, @checkPartialDetections);

            parser.parse(varargin{:});

            showProgressBar = parser.Results.ShowProgressBar;
            progressBarParent = parser.Results.ProgressBarParent;
            highDistortion  = parser.Results.HighDistortion;
            usePartial      = parser.Results.PartialDetections;
            minCornerMetric = parser.Results.MinCornerMetric;

            % Use 'IsDistortionHigh' property when the default value is
            % being used for HighDistortion
            if ismember('HighDistortion', parser.UsingDefaults)
                highDistortion = this.IsDistortionHigh;
            end

            if ismember('MinCornerMetric', parser.UsingDefaults)
                minCornerMetric = this.MinCornerMetric;
            end

            [imagePoints, boardSize, pairsUsed, userCanceled] = detectCheckerboardPoints(...
                imageFileNames1, imageFileNames2,...
                'HighDistortion', highDistortion,...
                'PartialDetections', usePartial, ...
                'MinCornerMetric', minCornerMetric, ...
                'ShowProgressBar', showProgressBar, ...
                'ProgressBarParent', progressBarParent);

            if userCanceled
                error(message('vision:uitools:LoadingCanceledByUser'));
            end

            this.BoardSize = boardSize;

            %--------------------------------------------------------------
            % Validation functions for command line workflow
            %--------------------------------------------------------------
            function tf = checkShowProgressBar(showProgressBar)
                validateattributes(showProgressBar, {'logical', 'numeric'},...
                    {'scalar'}, mfilename, 'ShowProgressBar');
                tf = true;
            end

            %--------------------------------------------------------------
            function tf = checkProgressBarParent(progressBarParent)
                if ~isempty(progressBarParent)
                    validateattributes(progressBarParent, {'matlab.ui.container.internal.AppContainer'},...
                        {'nonempty'}, mfilename, 'ProgressBarParent');
                end
                tf = true;
            end

            %--------------------------------------------------------------
            function tf = checkMinCornerMetric(value)
                validateattributes(value, {'single', 'double'},...
                    {'scalar', 'real', 'nonnegative', 'finite'}, mfilename, 'MinCornerMetric');
                tf = true;
            end

            %--------------------------------------------------------------
            function tf = checkHighDistortion(highDistortion)
                validateattributes(highDistortion, {'logical', 'numeric'},...
                    {'scalar','binary'}, mfilename, 'HighDistortion');
                tf = true;
            end

            %--------------------------------------------------------------
            function tf = checkPartialDetections(usePartial)
                validateattributes(usePartial, {'logical', 'numeric'},...
                    {'scalar','binary'}, mfilename, 'PartialDetections');
                tf = true;
            end

        end

        %------------------------------------------------------------------
        % World point generation
        %------------------------------------------------------------------
        function worldPoints = generateWorldPoints(this, varargin)

            % Handle command line usage (Name-Value pairs are used to
            % set properties inplace of the properties panel in the calibrator app)
            if nargin > 1 % Command Line workflow
                parser = inputParser;
                parser.addParameter('SquareSize', 25, @checkSquareSize);
                parser.addParameter('BoardSize', this.BoardSize, @checkBoardSize);
                parser.parse(varargin{:});

                boardSize = parser.Results.BoardSize;
                squareSize = parser.Results.SquareSize;
            else % Calibrator App workflow
                boardSize = this.BoardSize;
                squareSize = this.SquareSize;
            end

            worldPoints = generateCheckerboardPoints(boardSize, squareSize);

            %--------------------------------------------------------------
            % Validation functions for command line workflow
            %--------------------------------------------------------------
            function tf = checkSquareSize(squareSize)
                validateattributes(squareSize, {'numeric'}, ...
                    {'scalar', 'positive', 'finite', 'nonsparse'}, mfilename, 'SquareSize');
                tf = true;
            end

            %--------------------------------------------------------------
            function tf = checkBoardSize(boardSize)
                validateattributes(boardSize, {'numeric'},...
                    {'nonempty', 'vector', 'numel', 2, 'integer', 'positive', '>=', 3},....
                    mfilename, 'BoardSize');
                tf = true;
            end

        end

    end

    methods

        %------------------------------------------------------------------
        % Set up properties panel related to checkerboard detection
        %------------------------------------------------------------------
        function propertiesPanel(this, panel)
            this.Panel = panel;

            % Add and configure UI components int he properties panel
            configureUIComponents(this);

            % Initialize property values
            initializePropertyValues(this);
        end

        %------------------------------------------------------------------
        % Set up method for drawing axes labels
        %------------------------------------------------------------------
        function [originLabel, xLabel, yLabel] = drawImageAxesLabels(this, imagePoints)

            numBoardRows = this.BoardSize(1)-1;
            numBoardCols = this.BoardSize(2)-1;

            % Reshape checkerboard corners to boardSize shaped array
            boardCoordsX = reshape(imagePoints(:,1), [numBoardRows, numBoardCols]);
            boardCoordsY = reshape(imagePoints(:,2), [numBoardRows, numBoardCols]);
            boardCoords = cat(3, boardCoordsX, boardCoordsY);

            % Origin label (check if origin location is inside the image)
            if ~isnan(boardCoordsX(1,1))
                p1 = boardCoords(1,1,:);

                refPointIdx = find(~isnan(boardCoordsX(:,1)),2);
                p2 = boardCoords(refPointIdx(2),1,:);

                refPointIdx = find(~isnan(boardCoordsX(1,:)),2);
                p3 = boardCoords(1,refPointIdx(2),:);

                [loc, theta] = getAxesLabelPosition(p1, p2, p3);

                originLabel.Location    = loc;
                originLabel.Orientation = theta;
            else
                originLabel = struct;
            end

            % X-axis label
            firstRowIdx = numBoardCols:-1:1;
            refPointIdx13 = find(~isnan(boardCoordsX(1,firstRowIdx)), 2);
            refPointIdx13 = firstRowIdx(refPointIdx13);

            p1 = boardCoords(1,refPointIdx13(1),:);
            p3 = boardCoords(1,refPointIdx13(2),:);

            refPointIdx2 = find(~isnan(boardCoordsX(:,refPointIdx13(1))), 2);
            p2 = boardCoords(refPointIdx2(2),refPointIdx13(1),:);

            [loc, theta] = getAxesLabelPosition(p1, p2, p3);
            theta = 180 + theta;

            xLabel.Location    = loc;
            xLabel.Orientation = theta;

            % Y-axis label
            firstColIdx = numBoardRows:-1:1;
            refPointIdx12 = find(~isnan(boardCoordsX(firstColIdx,1)),2);
            refPointIdx12 = firstColIdx(refPointIdx12);

            p1 = boardCoords(refPointIdx12(1),1,:);
            p2 = boardCoords(refPointIdx12(2),1,:);

            refPointIdx3 = find(~isnan(boardCoordsX(refPointIdx12(1),:)), 2);
            p3 = boardCoords(refPointIdx12(1),refPointIdx3(2),:);

            [loc, theta] = getAxesLabelPosition(p1, p2, p3);

            yLabel.Location    = loc;
            yLabel.Orientation = theta;

            %--------------------------------------------------------------
            % p1+v
            %  \
            %   \     v1
            %    p1 ------ p2
            %    |
            % v2 |
            %    |
            %    p3
            function [loc, theta] = getAxesLabelPosition(p1, p2, p3)
                v1 = p3 - p1;
                theta = -atan2d(v1(2), v1(1));

                v2 = p2 - p1;
                v = -v1 - v2;
                d = hypot(v(1), v(2));
                minDist = 40;
                if d < minDist
                    v = (v / d) * minDist;
                end
                loc = p1 + v;
            end
        end
        %------------------------------------------------------------------
        % Identify the theme
        %------------------------------------------------------------------
        function tf = isDarkMode(this)
            hFig = ancestor(this.Panel,'figure');
            if ~isempty(hFig.Theme)
                theme = hFig.Theme.BaseColorStyle;
                tf = strcmpi(theme,'dark');
            else
                s = settings;
                mode = s.matlab.appearance.MATLABTheme.ActiveValue;
                tf = strcmpi(mode,'Dark');
            end
        end

    end

    methods (Access = private)

        %------------------------------------------------------------------
        function configureUIComponents(this)

            %--------------------------------------------------------------
            % UI components for square size selector
            %--------------------------------------------------------------

            initSquareSize = 25;
            initUnits = vision.getMessage('vision:caltool:millimeters');

            this.PropertyPanelGridLayout = uigridlayout(this.Panel);
            this.PropertyPanelGridLayout.RowHeight = {'fit'};
            this.PropertyPanelGridLayout.ColumnWidth = {'1x','fit'};

            propertyPanelGridSection = uigridlayout(this.PropertyPanelGridLayout,[5,3]);
            propertyPanelGridSection.RowHeight = {'fit', 30, 1, 'fit', 'fit'};
            propertyPanelGridSection.ColumnWidth = {'fit','1x','1x'};

            % Label
            uilabel('Parent',propertyPanelGridSection,...
                'HorizontalAlignment', 'left',...
                'Text', vision.getMessage('vision:caltool:SquareSize'));

            % Editbox
            this.SquareSizeEditBox = uieditfield('Parent', propertyPanelGridSection,...
                'Value', num2str(initSquareSize),...
                'ValueChangedFcn', @(~, ~) doSquareSizeChanged(this), ...
                'Tooltip', vision.getMessage('vision:caltool:SquareSizeToolTip'));

            % Units selection
            this.UnitsPopup = uidropdown('Parent', propertyPanelGridSection,...
                'Items', this.AvailableUnits,...
                'Value', initUnits, ...
                'ValueChangedFcn', @(~, ~) doUnitsChanged(this), ...
                'Tag', 'UnitsSelectorPopup', ...
                'Tooltip', getString(message('vision:caltool:SquareSizeUnitsToolTip')));

            %--------------------------------------------------------------
            % UI components for image distortion level selection
            %--------------------------------------------------------------

            % Label
            uilabel('Parent', propertyPanelGridSection,...
                'HorizontalAlignment', 'Left',...
                'Text', vision.getMessage('vision:caltool:ImageDistortion'));

            % Radio buttons
            this.RadioButtonGroup = uibuttongroup('Parent', propertyPanelGridSection, ...
                'Visible', 'on', ...
                'Units', 'pixels', ...
                'BorderType', 'none', ...
                'SelectionChangedFcn',@(~, ~) doDistortionChanged(this));
            this.RadioButtonGroup.Layout.Row = 2;
            this.RadioButtonGroup.Layout.Column = [2 3];

            position1 = [2,  0, 50,  20]; % 'Low' option
            position2 = [80, 0, 70, 20];  % 'High' option

            this.LowDistortionButton = uiradiobutton(this.RadioButtonGroup,...
                'Position',position1, ...
                'Text', vision.getMessage('vision:caltool:LowDistortion'), ...
                'Tag', 'LowDistortionButton');

            this.HighDistortionButton = uiradiobutton(this.RadioButtonGroup,...
                'Position',position2, ...
                'Text', vision.getMessage('vision:caltool:HighDistortion'), ...
                'Tag', 'HighDistortionButton');

            % Display visual guide.
            propertyImgGridLayout = uigridlayout(this.PropertyPanelGridLayout, ...
                'Padding', 2);
            propertyImgGridLayout.RowHeight = {'fit'};
            propertyImgGridLayout.ColumnWidth = {'fit'};

            if this.isDarkMode
                visualGuideFile = fullfile(toolboxdir('vision'),'vision', ...
                                     '+vision', '+internal', '+calibration', ...
                                     '+tool', 'CheckerboardVisualGuideDark.png');
            else
                visualGuideFile = fullfile(toolboxdir('vision'),'vision', ...
                                     '+vision', '+internal', '+calibration', ...
                                     '+tool', 'CheckerboardVisualGuideLight.png');

            end
            vision.internal.calibration.tool.displayPatternThumbnail(propertyImgGridLayout, ...
                visualGuideFile);

            addAdvancedPropertiesAccordion(this, propertyPanelGridSection);
        end

        function addAdvancedPropertiesAccordion(this, panel)
            accordionParent = matlab.ui.container.internal.Accordion('Parent', ...
                panel);
            accordionParent.Layout.Row = [4 5];
            accordionParent.Layout.Column = [1 3];
            accordionPanel = matlab.ui.container.internal.AccordionPanel(...
                'Parent', accordionParent, ...
                'Tag', 'AdvancedPropertiesAccordion',...
                'Title', getString(message('vision:caltool:AdvancedProperties')),...
                'Collapsed', true);
             
            % Create grid layout for the advanced properties components
            gridLayout = uigridlayout(accordionPanel, [1 4], 'Padding', 2);
            gridLayout.RowHeight = 30;
            gridLayout.ColumnWidth =  {'fit', 50, 'fit', 'fit'};

            % Min corner metric label
            label = uilabel('Parent', gridLayout, ...
                'Text', getString(message('vision:caltool:MinCornerMetric')), ...
                'HorizontalAlignment', 'left', ...
                'Tooltip', getString(message('vision:caltool:MinCornerMetricToolTip')));
            label.Layout.Column = 1;
            
            % Min corner metric editbox
            minCornerMetric = 0.15;
            this.MinCornerMetricEditBox = uieditfield(gridLayout, 'numeric', ...
                'Enable', 'off', 'Value', minCornerMetric, ...
                'ValueChangedFcn', @(~, ~) doMinCornerMetricChange(this), ...
                'Tag', 'MinCornerMetricEditBox');
            this.MinCornerMetricEditBox.Layout.Column = 2;

            % Min corner metric mode radio buttons
            this.CornerMetricModeRadioButtonGroup = uibuttongroup('Parent', gridLayout, ...
                'Visible', 'on', 'Units', 'pixels', 'BorderType', 'none', ...
                'SelectionChangedFcn',@(~, ~) doMinCornerMetricModeChange(this), ...
                'Tag', 'MinCornerMetricModeRadioButtons');
            this.CornerMetricModeRadioButtonGroup.Layout.Row = 1;
            this.CornerMetricModeRadioButtonGroup.Layout.Column = 3;

            position1 = [2  5 50 20]; % 'Auto' option
            position2 = [60 5 70 20]; % 'Manual' option

            this.AutoCornerMetricModeButton = uiradiobutton(this.CornerMetricModeRadioButtonGroup, ...
                'Position', position1, 'Text', getString(message('vision:caltool:Auto')), ...
                'Tag', 'AutoMinCornerMetricModeButton');
            this.ManualCornerMetricModeButton = uiradiobutton(this.CornerMetricModeRadioButtonGroup, ...
                'Position', position2, 'Text', getString(message('vision:caltool:Manual')), ...
                'Tag', 'ManualMinCornerMetricModeButton');
        end

    end

    %----------------------------------------------------------------------
    % Callbacks for UI components to update corresponding properties
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function initializePropertyValues(this)
            % SquareSize
            this.SquareSize = str2double(get(this.SquareSizeEditBox,'Value'));

            if this.SquareSize <= 0 || isnan(this.SquareSize)
                uialert(this.Panel,...
                    getString(message('vision:caltool:invalidSquareSize')),...
                    getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle')));
            end

            % Units
            this.WorldUnits = get(this.UnitsPopup,'value');

            % IsDistortionHigh
            this.IsDistortionHigh = this.HighDistortionButton.Value;

            % MinCornerMetric
            if this.IsDistortionHigh
                this.MinCornerMetric = 0.12;
            else
                this.MinCornerMetric = 0.15;
            end
        end

        %------------------------------------------------------------------
        function doSquareSizeChanged(this)
            this.SquareSize = str2double(get(this.SquareSizeEditBox,'Value'));

            if this.SquareSize <= 0 || isnan(this.SquareSize)
                uialert(this.Panel.Parent, getString(message('vision:caltool:invalidSquareSize')),...
                    getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle')));
            end
        end

        %------------------------------------------------------------------
        function doUnitsChanged(this)
            this.WorldUnits = get(this.UnitsPopup,'value');
        end

        %------------------------------------------------------------------
        function doDistortionChanged(this)

            this.IsDistortionHigh = this.HighDistortionButton.Value;
            if this.AutoCornerMetricModeButton.Value
                % Min corner metric mode is set to auto.
                if this.IsDistortionHigh
                    this.MinCornerMetricEditBox.Value = 0.12;
                    this.MinCornerMetric = this.MinCornerMetricEditBox.Value;
                else
                    this.MinCornerMetricEditBox.Value = 0.15;
                    this.MinCornerMetric = this.MinCornerMetricEditBox.Value;
                end
            end
        end

        %------------------------------------------------------------------
        function doMinCornerMetricChange(this)
            minCornerMetric = this.MinCornerMetricEditBox.Value;
            try
                paramName = lower(getString(message('vision:caltool:MinCornerMetric')));
                validateattributes(minCornerMetric, {'single', 'double'},...
                    {'scalar', 'real', 'nonnegative', 'finite'}, '', ...
                    paramName(1:end-1));
                this.MinCornerMetric = minCornerMetric;
            catch ME
                parent = this.Panel.Parent.Parent;
                uialert(parent, ME.message, ...
                    getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle')));
                this.MinCornerMetricEditBox.Value = this.MinCornerMetric;
            end
        end

        %------------------------------------------------------------------
        function doMinCornerMetricModeChange(this)
            if this.ManualCornerMetricModeButton.Value
                this.MinCornerMetricEditBox.Enable = 'on';
            else
                if this.IsDistortionHigh
                    this.MinCornerMetricEditBox.Value = 0.12;
                else
                    this.MinCornerMetricEditBox.Value = 0.15;
                end
                this.MinCornerMetricEditBox.Enable = 'off';
            end
        end        
    end
end
