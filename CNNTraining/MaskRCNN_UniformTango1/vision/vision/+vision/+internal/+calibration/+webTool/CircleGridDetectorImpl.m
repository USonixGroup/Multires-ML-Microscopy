classdef CircleGridDetectorImpl < vision.calibration.PatternDetector
    % CircleGridDetectorImpl Implements the interface for circle grid detection for camera calibration.
    %
    %   CircleGridDetectorImpl is an abstract base class for circle grid detectors
    %   in the Single and Stereo Camera Calibrator Apps. It implements the interface
    %   for these detectors to be used in the apps and from the command line.
    %
    %   For more information on defining a custom calibration pattern detector
    %   using this interface, see the PatternDetector class:
    %
    %       edit vision.calibration.PatternDetector

    % Copyright 2021-2024 The MathWorks, Inc.

    %----------------------------------------------------------------------
    properties(Constant, Abstract)
        Name

        PatternType
    end

    %----------------------------------------------------------------------
    properties
        Panel

        WorldUnits = vision.getMessage('vision:caltool:millimeters');

        PatternDims

        CenterDistance

        CircleColor = "black";
    end

    %----------------------------------------------------------------------
    % UI components for the properties panel.
    %----------------------------------------------------------------------
    properties (Access = private)
        CenterDistanceEditBox

        PatternDim1EditBox

        PatternDim2EditBox

        UnitsPopup

        RowColSelectorGridLayout

        CircleColorBlackButton
        
        CircleColorWhiteButton

        PropertyImgGridLayout

        ParentGridLayout
    end

    properties (Access = private, Constant)
        AvailableUnits = vision.internal.calibration.availableWorldUnits;
    end

    properties(Constant)
        SymmetricCircleGridLight = fullfile(toolboxdir('vision'),'vision', ...
                        '+vision', '+internal', '+calibration', ...
                        '+tool', 'SymmetricCBVisualGuideLight.png');

        SymmetricCricleGridDark = fullfile(toolboxdir('vision'),'vision', ...
                        '+vision', '+internal', '+calibration', ...
                        '+tool', 'SymmetricCBVisualGuideDark.png');

        AsymmetricCricleGridLight = fullfile(toolboxdir('vision'),'vision', ...
                        '+vision', '+internal', '+calibration', ...
                        '+tool', 'AsymmetricCBVisualGuideLight.png');

        AsymmetricCircleGridDark = fullfile(toolboxdir('vision'),'vision', ...
                        '+vision', '+internal', '+calibration', ...
                        '+tool', 'AsymmetricCBVisualGuideDark.png');
    end

    methods

        %------------------------------------------------------------------
        % Detection method for pattern keypoint detection in the calibration
        % images.
        %------------------------------------------------------------------
        function [imagePoints, imagesUsed] = detectPatternPoints(this, varargin)

            if isa(this, 'vision.calibration.stereo.AsymmetricCircleGridDetector')
                imageFileNames1 = varargin{1};
                imageFileNames2 = varargin{2};
                nvPairs         = varargin(3:end);
            else
                imageFileNames1 = varargin{1};
                nvPairs         = varargin(2:end);
            end

            % Handle command line usage (Name-Value pairs are used to set
            % properties inplace of the properties panel in the calibrator app).
            parser = inputParser;
            parser.addParameter('ShowProgressBar', true, @checkShowProgressBar);
            parser.addParameter('ProgressBarParent', [], @checkProgressBarParent);
            parser.addParameter('CircleColor', this.CircleColor, @checkCircleColor);
            parser.addParameter('PatternDims', this.PatternDims, @checkPatternDims);
            parser.parse(nvPairs{:});

            showProgressBar = parser.Results.ShowProgressBar;
            progressBarParent = parser.Results.ProgressBarParent;
            circleColor = parser.Results.CircleColor;
            patternDims = parser.Results.PatternDims;
            patternType = this.PatternType;

            if isa(this, 'vision.calibration.stereo.AsymmetricCircleGridDetector')
                [imagePoints, imagesUsed, userCanceled] = detectCircleGridPoints(...
                    imageFileNames1, imageFileNames2, patternDims,...
                    'PatternType', patternType,...
                    'CircleColor', circleColor, ...
                    'ShowProgressBar', showProgressBar,...
                    'ProgressBarParent', progressBarParent);
            else
                [imagePoints, imagesUsed, userCanceled] = detectCircleGridPoints(...
                    imageFileNames1, patternDims, 'PatternType', patternType,...
                    'CircleColor', circleColor, ...
                    'ShowProgressBar', showProgressBar,...
                    'ProgressBarParent', progressBarParent);
            end

            if userCanceled
                error(message('vision:uitools:LoadingCanceledByUser'));
            end

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
            function tf = checkCircleColor(circleColor)
                validatestring(circleColor, ["black", "white"], ...
                    mfilename, 'CircleColor');
                tf = true;
            end
        end

        %--------------------------------------------------------------
        % Identify the theme
        %--------------------------------------------------------------
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

        %------------------------------------------------------------------
        % World point generation.
        %------------------------------------------------------------------
        function worldPoints = generateWorldPoints(this, varargin)

            % Handle command line usage (Name-Value pairs are used to set
            % properties inplace of the properties panel in the calibrator app).
            if nargin > 1 % Command Line workflow
                parser = inputParser;
                parser.addParameter('CenterDistance', 25, @checkCenterDistance);
                parser.addParameter('PatternDims', this.PatternDims, @checkPatternDims);
                parser.parse(varargin{:});

                centerDistance  = parser.Results.CenterDistance;
                patternDims     = parser.Results.PatternDims;
            else % Calibrator App workflow
                centerDistance  = this.CenterDistance;
                patternDims     = this.PatternDims;
            end

            patternType = this.PatternType;
            worldPoints = generateCircleGridPoints(patternDims, centerDistance, ...
                'PatternType', patternType);

            %--------------------------------------------------------------
            function tf = checkCenterDistance(centerDistance)
                validateattributes(centerDistance, {'numeric'}, ...
                    {'scalar', 'positive', 'finite', 'nonsparse'}, ...
                    mfilename, 'CenterDistance');
                tf = true;
            end
        end
    end

    methods

        %------------------------------------------------------------------
        % Set up properties panel related to CircleGrid detection.
        %------------------------------------------------------------------
        function propertiesPanel(this, panel)
            this.Panel = panel;

            % Add and configure UI components int he properties panel.
            configureUIComponents(this);

            % Initialize property values.
            initializePropertyValues(this);
        end

        %------------------------------------------------------------------
        % Set up method for drawing axes labels.
        %------------------------------------------------------------------
        function [originLabel, xLabel, yLabel] = drawImageAxesLabels(this, imagePoints)

            if strcmp(this.PatternType, 'asymmetric')
                % The reference points for each label are two columns/rows apart
                % instead of one row/column for symmetric. This is because the
                % rows are offset by half a column in asymmetric circle grid.
                refPtIdxOffset = 2;
            else
                refPtIdxOffset = 1;
            end

            [originLabel, xLabel, yLabel] = ...
                vision.internal.calibration.tool.getAxesLabelPositions(...
                imagePoints, this.PatternDims(1), this.PatternDims(2), refPtIdxOffset);
        end
    end

    %----------------------------------------------------------------------
    % Helpers to configure UI components on the properties panel.
    %----------------------------------------------------------------------
    methods (Access = private)

        %------------------------------------------------------------------
        function configureUIComponents(this)

            if isa(this, 'vision.calibration.monocular.SymmetricCircleGridDetector')
                % Configure UI components for pattern dimension selector.
                initPatternDims = [5 4];
                configureNumRowsNumColsSelector(this, initPatternDims);

                if this.isDarkMode
                    visualGuideFile = this.SymmetricCricleGridDark;
                else
                    visualGuideFile = this.SymmetricCircleGridLight;
                end
            else
                % Configure UI components for pattern dimension selector.
                initPatternDims = [3 5];
                configurePatternDimsSelector(this, initPatternDims);

                % Configure UI components for pattern dimensions help.
                configurePatternDimsInfoButton(this);
                
                 if this.isDarkMode
                    visualGuideFile = this.AsymmetricCircleGridDark;
                else
                    visualGuideFile = this.AsymmetricCricleGridLight;
                end
            end

            % Configure UI components for center distance selector.
            initCenterDistance = 25;
            initUnits = vision.getMessage('vision:caltool:millimeters');
            configureCenterDistanceSelector(this, ...
                initCenterDistance, initUnits);

            % Configure UI components for circle color.
            configureCircleColorSelector(this);

            % Display visual guide.
            vision.internal.calibration.tool.displayPatternThumbnail(...
                this.PropertyImgGridLayout, visualGuideFile);
        end

        %------------------------------------------------------------------
        function configureCenterDistanceSelector(this, initCenterDistance, initUnits)

            distanceSelectorGridLayout = uigridlayout(this.ParentGridLayout, 'Padding', 2);
            distanceSelectorGridLayout.RowHeight = {'fit'};
            distanceSelectorGridLayout.ColumnWidth = {'fit','1x','1x'};
            distanceSelectorGridLayout.Layout.Row = 1;

            % Label
            centerDistLabel = uilabel('Parent', distanceSelectorGridLayout, ...
                'Text', vision.getMessage('vision:caltool:CenterDistance'),...
                'HorizontalAlignment', 'left');
            centerDistLabel.Layout.Column = 1;

            % Editbox
            this.CenterDistanceEditBox = uieditfield(distanceSelectorGridLayout,...
                'Editable', 'on', ...
                'HorizontalAlignment', 'left', ...
                'Tag', 'CenterDistanceEditbox', ...
                'Value', num2str(initCenterDistance), ...
                'ValueChangedFcn', @(~, ~) doCenterDistanceChanged(this), ...
                'Tooltip', vision.getMessage('vision:caltool:CenterDistanceToolTip'));
            this.CenterDistanceEditBox.Layout.Column = 2;

            % Units selection
            this.UnitsPopup = uidropdown('Parent', distanceSelectorGridLayout,...
                'Items', this.AvailableUnits,...
                'Value', initUnits, ...
                'ValueChangedFcn', @(~, ~) doUnitsChanged(this), ...
                'Tag', 'UnitsSelectorPopup', ...
                'Tooltip', getString(message('vision:caltool:CenterDistanceUnitsToolTip')));
            this.UnitsPopup.Layout.Column = 3;
        end

        %------------------------------------------------------------------
        function configureCircleColorSelector(this)
            circleColorSelectorGridLayout = uigridlayout(this.ParentGridLayout, 'Padding', 2);
            circleColorSelectorGridLayout.RowHeight = {20};
            circleColorSelectorGridLayout.ColumnWidth = {'fit', 10, 'fit'};
            circleColorSelectorGridLayout.Layout.Row = 3;

            circleColorTxt = getString(message('vision:caltool:CircleColor'));
            label = uilabel(circleColorSelectorGridLayout,...
                'Text', circleColorTxt, 'HorizontalAlignment', 'left', ...
                'Tooltip', getString(message('vision:caltool:CircleColorTooltip')));
            label.Layout.Row = 1;
            label.Layout.Column = 1;

            circleColorRadioButtons = uibuttongroup('Parent', circleColorSelectorGridLayout, ...
                'Visible', 'on', 'Units', 'pixels', 'BorderType', 'none', ...
                'SelectionChangedFcn', @(~, ~)doCircleColorChanged(this), ...
                'Tag', 'CircleColorSelectorButtons');
            circleColorRadioButtons.Layout.Row = 1;
            circleColorRadioButtons.Layout.Column = 3;

            position1 = [2 0 50 20]; % 'Black' option
            position2 = [80 0 70 20];  % 'White' option

            blackTxt = getString(message('vision:caltool:Black'));
            this.CircleColorBlackButton = uiradiobutton(circleColorRadioButtons, ...
                'Position', position1, 'Text', blackTxt, 'Tag', 'CircleColorBlackButton');

            whiteTxt = getString(message('vision:caltool:White'));
            this.CircleColorWhiteButton = uiradiobutton(circleColorRadioButtons, ...
                'Position', position2, 'Text', whiteTxt, 'Tag', 'CircleColorWhiteButton');
        end

        %------------------------------------------------------------------
        function configurePatternDimsSelector(this, initPatternDims)

            % Create Grid Layout to arrange the components in uiFigure
            this.ParentGridLayout = uigridlayout(this.Panel);
            this.ParentGridLayout.RowHeight = {'fit','fit'};
            this.ParentGridLayout.ColumnWidth = {'3x','1x'};

            this.RowColSelectorGridLayout = uigridlayout(this.ParentGridLayout, 'Padding', 2);
            this.RowColSelectorGridLayout.RowHeight = {20};
            this.RowColSelectorGridLayout.ColumnWidth = {'fit', 20 ,'1x', '1x', '1x', '1x'};
            this.RowColSelectorGridLayout.Layout.Row = 2;

            this.PropertyImgGridLayout = uigridlayout(this.ParentGridLayout, 'Padding', 2);
            this.PropertyImgGridLayout.RowHeight = {'fit'};
            this.PropertyImgGridLayout.ColumnWidth = {'fit'};
            this.PropertyImgGridLayout.Layout.Row = [1 2];

            % Label
            patternDimsLabel = uilabel('Parent', this.RowColSelectorGridLayout, ...
                'Text', vision.getMessage('vision:caltool:PatternDimsLabel'));
            patternDimsLabel.Layout.Column= 1;

            % Label
            dia1Label = uilabel('Parent',  this.RowColSelectorGridLayout, ...
                'Text', vision.getMessage('vision:caltool:PatternDim1Label'));
            dia1Label.Layout.Column= 3;

            % Editbox
            this.PatternDim1EditBox = uieditfield( this.RowColSelectorGridLayout,...
                'Editable', 'on', ...
                'Tag', 'PatternDim1Editbox', ...
                'Value', num2str(initPatternDims(1)), ...
                'ValueChangedFcn', @(~, ~) doPatternDim1Changed(this), ...
                'Tooltip', vision.getMessage('vision:caltool:PatternDim1ToolTip'));
            this.PatternDim1EditBox.Layout.Column= 4;

            % Label
            dia2Label = uilabel('Parent',  this.RowColSelectorGridLayout, ...
                'Text', vision.getMessage('vision:caltool:PatternDim2Label'),...
                'HorizontalAlignment', 'left');
            dia2Label.Layout.Column= 5;

            % Editbox
            this.PatternDim2EditBox = uieditfield(this.RowColSelectorGridLayout,...
                'Editable', 'on', ...
                'HorizontalAlignment', 'left',...
                'Tag', 'PatternDim2Editbox', ...
                'Value', num2str(initPatternDims(2)), ...
                'ValueChangedFcn', @(~, ~) doPatternDim2Changed(this), ...
                'Tooltip', vision.getMessage('vision:caltool:PatternDim2ToolTip'));
            this.PatternDim2EditBox.Layout.Column = 6;
        end

        %------------------------------------------------------------------
        function configureNumRowsNumColsSelector(this, initPatternDims)

            % Create Grid Layout to arrange the components in uiFigure
            this.ParentGridLayout = uigridlayout(this.Panel);
            this.ParentGridLayout.RowHeight = {'fit','fit'};
            this.ParentGridLayout.ColumnWidth = {'3x','1x'};

            this.RowColSelectorGridLayout = uigridlayout(this.ParentGridLayout, 'Padding', 2);
            this.RowColSelectorGridLayout.RowHeight = {'fit'};
            this.RowColSelectorGridLayout.ColumnWidth =  {'fit', '1x', 'fit', '1x'};
            this.RowColSelectorGridLayout.Layout.Row = 2;

            this.PropertyImgGridLayout = uigridlayout(this.ParentGridLayout, 'Padding', 2);
            this.PropertyImgGridLayout.RowHeight = {'fit'};
            this.PropertyImgGridLayout.ColumnWidth = {'fit'};
            this.PropertyImgGridLayout.Layout.Row = [1 2];

            % Label
            numRowlabel = uilabel('Parent', this.RowColSelectorGridLayout, ...
                'Text', vision.getMessage('vision:caltool:NumRowsLabel'),...
                'HorizontalAlignment', 'left');
            numRowlabel.Layout.Column= 1;

            % Editbox
            this.PatternDim1EditBox = uieditfield(this.RowColSelectorGridLayout,...
                'Editable', 'on', ...
                'Tag', 'PatternDim1Editbox', ...
                'Value', num2str(initPatternDims(1)), ...
                'ValueChangedFcn', @(~, ~) doPatternDim1Changed(this), ...
                'Tooltip', vision.getMessage('vision:caltool:NumRowsToolTip'));
            this.PatternDim1EditBox.Layout.Column= 2;

            % Label
            numColsLabel = uilabel('Parent', this.RowColSelectorGridLayout, ...
                'Text', vision.getMessage('vision:caltool:NumColsLabel'),...
                'HorizontalAlignment', 'left');
            numColsLabel.Layout.Column= 3;

            % Editbox
            this.PatternDim2EditBox = uieditfield(this.RowColSelectorGridLayout,...
                'Editable', 'on', ...
                'Tag', 'PatternDim2Editbox', ...
                'Value', num2str(initPatternDims(2)), ...
                'ValueChangedFcn', @(~, ~) doPatternDim2Changed(this), ...
                'Tooltip', vision.getMessage('vision:caltool:NumColsToolTip'));
            this.PatternDim2EditBox.Layout.Column= 4;
        end

        %------------------------------------------------------------------
        function configurePatternDimsInfoButton(this)
  
            helpIcon = uiimage(this.RowColSelectorGridLayout,...
                'ImageClickedFcn', @(~, ~) openDoc(this),...
                'Tag','PatternDimsHelp', ...
                'Tooltip', vision.getMessage('vision:caltool:PatternDimsHelp'));
            helpIcon.Layout.Row = 1;
            helpIcon.Layout.Column= 2;
            matlab.ui.control.internal.specifyIconID...
             (helpIcon, 'infoUI', 16); 
        end
    end

    %----------------------------------------------------------------------
    % Callbacks for UI components to update corresponding properties.
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function initializePropertyValues(this)

            % CenterDistance
            this.CenterDistance = str2double(get(this.CenterDistanceEditBox,'Value'));

            % PatternDims
            patternDim1 = str2double(get(this.PatternDim1EditBox,'Value'));
            patternDim2  = str2double(get(this.PatternDim2EditBox,'Value'));
            this.PatternDims = [patternDim1 patternDim2];

            % Units
            this.WorldUnits = get(this.UnitsPopup,'value');
        end

        %------------------------------------------------------------------
        function doCenterDistanceChanged(this)
            this.CenterDistance = str2double(get(this.CenterDistanceEditBox,'Value'));
            if this.CenterDistance <= 0 || isnan(this.CenterDistance)
                parent = ancestor(this.Panel,'Figure');
                errorMsg = getString(message('vision:caltool:invalidCenterDistance'));
                title = getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle'));

                uialert(parent, errorMsg, title);

            end
        end

        %------------------------------------------------------------------
        function doPatternDim1Changed(this)
            this.PatternDims(1) = str2double(get(this.PatternDim1EditBox,'Value'));
            dimension = 1;
            validatePatternDims(this, dimension);
        end

        %------------------------------------------------------------------
        function doPatternDim2Changed(this)
            this.PatternDims(2) = str2double(get(this.PatternDim2EditBox,'Value'));
            dimension = 2;
            validatePatternDims(this, dimension);
        end

        %------------------------------------------------------------------
        % Pattern dimensions validator for calibrator app workflow.
        %------------------------------------------------------------------
        function validatePatternDims(this, idx)

            if strcmp(this.PatternType, "symmetric")
                msgId = 'vision:caltool:invalidPatternSize';
            else
                msgId = 'vision:caltool:invalidPatternDims';
            end

            isNotNumericPositiveInteger = @(x) isnan(x) || x <= 0 || x ~= floor(x);
            if isNotNumericPositiveInteger(this.PatternDims(idx))
                parent = ancestor(this.Panel,'Figure');
                errorMsg = vision.getMessage(msgId);
                title = getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle'));

                uialert(parent, errorMsg, title);
            end

            if strcmp(this.PatternType, "asymmetric") && all(mod(this.PatternDims, 2) == 0)
                % Error if inputs are for a 180-degrees rotation variant asymmetric
                % circle grid.
                parent = ancestor(this.Panel,'Figure');
                errorMsg = getString(message('vision:calibrate:unsupportedPatternDims'));
                title = getString(message('MATLAB:uistring:popupdialogs:ErrorDialogTitle'));

                uialert(parent, errorMsg, title);
            end
        end

        %------------------------------------------------------------------
        function doUnitsChanged(this)
            this.WorldUnits = get(this.UnitsPopup,'value');
        end

        %------------------------------------------------------------------
        function doCircleColorChanged(this)
            if this.CircleColorWhiteButton.Value
                this.CircleColor = "white";
            else
                this.CircleColor = "black";
            end
        end

        %------------------------------------------------------------------
        function openDoc(~)
            helpview('vision','circleGridPatterns');
        end
    end
end

%----------------------------------------------------------------------------
% PatternDims validator for detector's and generator's command line workflow.
%----------------------------------------------------------------------------
function tf = checkPatternDims(patternDims)
validateattributes(patternDims, {'numeric'},...
    {'nonempty', 'vector', 'numel', 2, 'integer', 'positive', '>=', 3},...
    mfilename, 'PatternDims');
tf = true;
end
