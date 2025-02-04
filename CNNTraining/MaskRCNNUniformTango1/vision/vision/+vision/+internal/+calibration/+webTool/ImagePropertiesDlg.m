% ImagePropertiesDlg Dialog for getting square size and distortion level

% Copyright 2014-2024 The MathWorks, Inc.

classdef ImagePropertiesDlg < images.internal.app.utilities.Dialog

    properties
        PropertiesPanel
        DefaultPropPos
        
        PatternSelector
        CalibratorTool

        Canceled
    end
    
    properties (Access = private)
        InitDlgLocation;
        CustomPatternPanel;
        CustomPatternPanelHeightOffset;
        
        RightArrowImage;
        DownArrowImage;
        CustomPatternExpandButton;
        IsCustomPatternExpanded = false;

        ParentGridLayout
        OkCancelGridLyout
        Ok
        Cancel
    end
    
    properties (Dependent)
        CurrentDetector

        CurrentDetectorFileName
    end
    
    methods
        %------------------------------------------------------------------
        function this = ImagePropertiesDlg(tool, location)
            dlgTitle = vision.getMessage('vision:caltool:ImagePropsTitle');
            this = this@images.internal.app.utilities.Dialog(location, dlgTitle);

            this.CalibratorTool = tool;
            
            this.Size = [520, 545];
            create(this);
            this.FigureHandle.Visible = 'on';

            % Workaround to get modal behavior in linux machines g2126774
            if ~ispc() && ~ismac() && ~isempty(this.CalibratorTool)
                set(this.CalibratorTool.AppContainer, 'Busy', 1);
                this.FigureHandle.DeleteFcn = @(~,~)resumeAppContainerWaitState(this);
            end
            
             % Set windows down to react to restored focus
            this.FigureHandle.WindowButtonDownFcn = @(~,~)reactToRestoredFocus(this);
            this.FigureHandle.WindowButtonMotionFcn = @(~,~)reactToGainedFocus(this);
            
            this.ParentGridLayout = uigridlayout(this.FigureHandle);
            this.ParentGridLayout.RowHeight = {'fit', 'fit', 'fit', '1x', 'fit'};
            this.ParentGridLayout.ColumnWidth = {500};

             % Add pattern selector options
            this.PatternSelector = vision.internal.calibration.tool.PatternSelector(...
                   this, this.ParentGridLayout, [], vision.getMessage('vision:caltool:DefaultPattern'));
            
            % Add custom pattern panel
            this.CustomPatternPanel = vision.internal.calibration.webTool.CustomPatternPanel(...
                   this.ParentGridLayout);
            
            % Link pattern selector to custom pattern panel.
            linkPatternSelector(this.CustomPatternPanel, this.PatternSelector);
            
            % Add properties panel
            setupPropertiesPanel(this);

            % Set default pattern (checkerboard) and related properties
            updateDialog(this, vision.getMessage('vision:caltool:DefaultPattern'));

            this.addOkCancelButton();
            this.UpdateFigureSize();
            this.FigureHandle.ThemeChangedFcn = @(src,evt)this.updateOnThemeChange(src,evt);
        end
        
        %------------------------------------------------------------------
        function setupPropertiesPanel(this)
            this.PropertiesPanel = uipanel('Parent', this.ParentGridLayout,...
                'Title', vision.getMessage('vision:caltool:PropertiesPanelTitle'), ...
                'Units', 'pixels',...
                'Tag','PropertiesPanel');
        end
        
        %------------------------------------------------------------------
        function updateDialog(this, patternName)
            % Find pattern index
            selection = find(strcmp(this.PatternSelector.AvailablePatterns, patternName), 1);
            
            delete(this.PropertiesPanel.Children);
            
            % Populate the properties panel by invoking pattern detector method
            try
                patternDetector = this.PatternSelector.PatternDetectors{selection};
                isFiducialTarget = isa(patternDetector, 'vision.calibration.monocular.CharucoBoardDetector') ...
                                || isa(patternDetector, 'vision.calibration.monocular.AprilGridDetector');
                 if isFiducialTarget
                    this.PropertiesPanel.BorderType = 'none';
                    this.PropertiesPanel.Title = '';
                else
                    this.PropertiesPanel.BorderType = 'line';
                    this.PropertiesPanel.Title = vision.getMessage('vision:caltool:PropertiesPanelTitle');
                 end
                  propertiesPanel(patternDetector, this.PropertiesPanel);
            catch invokePropExp
                errorMsg = sprintf("%s: \n%s", vision.getMessage('vision:caltool:PropPanelErrorPreText'), ...
                    invokePropExp.message);
                uialert(this.FigureHandle, errorMsg, ...
                    vision.getMessage('vision:caltool:PropPanelFailedTitle'));
            end

            updatePatternSelector(this.PatternSelector, selection);
            this.UpdateFigureSize();
        end
                
        %------------------------------------------------------------------
        function detector = get.CurrentDetector(this)
           detector = this.PatternSelector.CurrentDetector; 
        end
        
        %------------------------------------------------------------------
        function fileName = get.CurrentDetectorFileName(this)
           fileName = this.PatternSelector.CurrentDetectorFileName; 
        end
        
        %------------------------------------------------------------------
        function updateDialogSize(this, dlgSize)
            this.Size = dlgSize;
            this.FigureHandle.Position(3) = dlgSize(1);
            this.FigureHandle.Position(4) = dlgSize(2);
        end
        
         %------------------------------------------------------------------
        function selectCurrentDetector(this)
            selectCurrentDetector(this.PatternSelector);
        end

        %------------------------------------------------------------------
        function updateOnThemeChange(this,~,~)
            if ~isempty(this.PropertiesPanel)
                % Update the checkerboard images
                s = settings;
                patternName = s.vision.calibrator.CurrentPattern.ActiveValue;
                updateDialog(this, patternName);
            end
        end
    end

     methods (Access = protected)
        %--Key Press-------------------------------------------------------
        function keyPress(this, evt)
            
            if ~validateKeyPressSupport(this,evt)
                return;
            end
            
            switch(evt.Key)
                case {'return','space'}
                    okClicked(this);
                case 'escape'
                    cancelClicked(this);
            end
            
        end
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function okClicked(this, ~, ~)
            this.Canceled = false;
            if ~isempty(this.PatternSelector)
                updateUnsavedPatternTemplates(this.PatternSelector);
            end
            
            close(this);
            
            if ~isempty(this.CalibratorTool)
                evtData = vision.internal.calibration.tool.PatterDetectorEventData(this.CurrentDetector, this.CurrentDetectorFileName);
                notify(this.CalibratorTool, 'ImagePropertiesSet', evtData);
            end
        end
        
        %------------------------------------------------------------------
        function cancelClicked(this, ~, ~)
            this.Canceled = true;
            close(this);
            
            if ~isempty(this.CalibratorTool)
                notify(this.CalibratorTool, 'ImagePropertiesClosed');
            end
        end

        %------------------------------------------------------------------
        function resumeAppContainerWaitState(this, ~, ~)
            if ~isempty(this.CalibratorTool)
                set(this.CalibratorTool.AppContainer,'Busy',0);
            end
        end
        
        %------------------------------------------------------------------
        function reactToRestoredFocus(this)
            if ~isempty(this.PatternSelector)
                updateUnsavedPatternTemplates(this.PatternSelector);
            end

            if strcmp(this.FigureHandle.WindowStyle, 'normal')
                this.FigureHandle.WindowStyle = 'modal';
            end
        end

        %------------------------------------------------------------------
        function reactToGainedFocus(this)
            if ~isempty(this.PatternSelector)
                updateUnsavedPatternTemplates(this.PatternSelector);
            end
        end

        function addOkCancelButton(this)

            this.OkCancelGridLyout = uigridlayout(this.ParentGridLayout);
            this.OkCancelGridLyout.RowHeight = {'fit'};
            this.OkCancelGridLyout.ColumnWidth = {'1x',80, 80};

            addOK(this);
            addCancel(this);
        end

        %--Add Ok----------------------------------------------------------
        function addOK(this)

            this.Ok =  uibutton('Parent',this.OkCancelGridLyout, ...
                'ButtonPushedFcn', @(~,~) okClicked(this),...
                'FontSize', 12, ...
                'Text',getString(message('MATLAB:uistring:popupdialogs:OK')),...
                'Tag', 'OK');

            this.Ok.Layout.Column = 2;
        end

        %--Add Cancel------------------------------------------------------
        function addCancel(this)

            this.Cancel = uibutton('Parent', this.OkCancelGridLyout, ...
                'ButtonPushedFcn', @(~,~) cancelClicked(this),...
                'FontSize', 12,...
                'Text',getString(message('MATLAB:uistring:popupdialogs:Cancel')),...
                'Tag', 'Cancel');

            this.Cancel.Layout.Column = 3;
        end

        function UpdateFigureSize(this)
            matlab.ui.internal.PositionUtils.fitToContent(this.FigureHandle, 'topleft');
        end
    end
end