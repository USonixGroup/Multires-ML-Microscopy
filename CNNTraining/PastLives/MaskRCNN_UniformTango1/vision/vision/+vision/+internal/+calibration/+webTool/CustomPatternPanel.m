% CustomPatternPanel A compound UI control for importing custom patterns.

% Copyright 2021-2023 The MathWorks, Inc.

classdef CustomPatternPanel < handle
    properties (Access = private)

        CustomPatternGridLayout

        Parent
        PatternSelector
        AccordianPanel
        ImportPatternButton
        ImportPatternLabel
        CreatePatternLabel
        CreatePatternHelpButton
    end

    properties (Dependent)
        Position
    end

    methods
        %------------------------------------------------------------------
        function this = CustomPatternPanel(gridParent)

            addPanel(this, gridParent);
            addImportPatternButton(this);
            addInfoButton(this);

            hideComponents(this);
        end

        %------------------------------------------------------------------
        function showComponents(this)
            this.AccordianPanel.Collapsed = false;
        end

        %------------------------------------------------------------------
        function hideComponents(this)
            this.AccordianPanel.Collapsed = true;
        end

        %------------------------------------------------------------------
        function disable(this)
            this.ImportPatternButton.Enable = 'off';
        end

        %------------------------------------------------------------------
        function linkPatternSelector(this, patternSelector)
            this.PatternSelector = patternSelector;
        end
    end

    methods
        %------------------------------------------------------------------
        function location = get.Position(this)
            location = this.AccordianPanel.Position;
        end

        %------------------------------------------------------------------
        function set.Position(this, location)
            this.AccordianPanel.Position = location;
        end
    end

    methods (Access=private)
        %------------------------------------------------------------------
        function addPanel(this, gridParent)
            accordianParent = matlab.ui.container.internal.Accordion('Parent', gridParent);
            this.AccordianPanel = matlab.ui.container.internal.AccordionPanel(...
                'Parent', accordianParent, ...
                'Tag', 'CustomPatternPanel',...
                'CollapsedChangedFcn', @this.updateUI,...
                'Title', vision.getMessage('vision:caltool:CustomPatternPanelTitle'));

            this.CustomPatternGridLayout = uigridlayout(this.AccordianPanel);
            this.CustomPatternGridLayout.ColumnWidth = {'1x',20};
            this.CustomPatternGridLayout.RowHeight = {'1x','fit', 30};
        end

        %------------------------------------------------------------------
        function addImportPatternButton(this)
            labelMsg = vision.getMessage('vision:caltool:ImportPatternDetectorLabel');

            this.ImportPatternLabel = uilabel(this.CustomPatternGridLayout, ...
                'Text', labelMsg, ...
                'Tag', 'TextMessage');

            this.ImportPatternLabel.WordWrap = 'on';
            this.ImportPatternLabel.Layout.Row = 1;
            this.ImportPatternLabel.Layout.Column = 1:2;

            this.ImportPatternButton = uibutton(this.CustomPatternGridLayout, ...
                'Tooltip', vision.getMessage('vision:caltool:ImportPatternTooltip'),...
                'Tag', 'ImportPatternButton', ...
                'ButtonPushedFcn', @this.doImportPatternSelected, ...
                'Text', vision.getMessage('vision:caltool:ImportPatternDetector'));

            this.ImportPatternButton.Layout.Row = 2;
            this.ImportPatternButton.Layout.Column = 1:2;
        end

        %------------------------------------------------------------------
        function addInfoButton(this)
            labelMsg = vision.getMessage('vision:caltool:InfoButtonLabel');
           
            this.CreatePatternLabel = uilabel('Parent', this.CustomPatternGridLayout, ...
                'Text', labelMsg, 'HorizontalAlignment', 'left', ...
                'Tag', 'TextMessage');
            this.CreatePatternLabel.WordWrap = 'on';
            this.CreatePatternLabel.Layout.Column = 1;
            this.CreatePatternLabel.Layout.Row = 3;
           
            this.CreatePatternHelpButton = uiimage(this.CustomPatternGridLayout,...
                'ImageClickedFcn', @(~, ~) openDoc(this),...
                'Tag','CreatePatternHelp',...
                'HorizontalAlignment','center');          
            matlab.ui.control.internal.specifyIconID...
             (this.CreatePatternHelpButton, 'infoUI', 16); 
            this.CreatePatternHelpButton.Layout.Column = 2;
            this.CreatePatternHelpButton.Layout.Row = 3;
        end

        %------------------------------------------------------------------
        function doImportPatternSelected(this, ~, ~)
            [file, path] = uigetfile('*.m', vision.getMessage('vision:caltool:SelectDetectorFileMessage'));

            if ~isequal(file, 0)
                fullPath = fullfile(path, file);

                % Get filename accounting for MATLAB style packaging
                [~, fileName] = ...
                    vision.internal.calibration.tool.PatternSelector.getFileParts(fullPath);

                addAndSelectImportedPattern(this.PatternSelector, fullPath, fileName);
            end
        end

        %------------------------------------------------------------------
        function openDoc(~)
            helpview('vision','createPatternDetectorHelp');
        end
    end

    methods (Static)
        %------------------------------------------------------------------
        function updateUI(accPanel, ~, ~)
            parent = ancestor(accPanel,'figure');
            matlab.ui.internal.PositionUtils.fitToContent(parent, 'topleft');
        end
    end
end