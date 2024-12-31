% LoadStereoImagesDlg Dialog for loading stereo images.

% Copyright 2014-2024 The MathWorks, Inc.

classdef LoadStereoImagesDlg < images.internal.app.utilities.Dialog
    properties
        Dir1 = '';
        Dir2 = '';
        
        FileNames = {};
        
        PropertiesPanel;
        ParentGridLayout;
        OkCancelGridLyout
        
        PatternSelector;
        CalibratorTool;
        IsFileMode;
    end
    
    properties (Access = private)
        CustomPatternPanel;
    end
    
    properties (Dependent)
        CurrentDetector;
        
        CurrentDetectorFileName;
    end
    
    properties(Access=private)        
        DirSelector1;
        DirSelector2;
        SizeSelector;
        Ok
        Cancel
    end
    
    methods
        function this = LoadStereoImagesDlg(tool, location, initialDir1, ...
                initialDir2)
            if nargin < 3
                initialDir1 = pwd();
            end
            
            if nargin < 4
                initialDir2 = pwd();
            end
            
            dlgTitle = vision.getMessage('vision:caltool:LoadStereoImagesTitle');
            this = this@images.internal.app.utilities.Dialog(location, dlgTitle);
            
            this.IsFileMode = false;
            this.CalibratorTool = tool;
            
            this.Size = [520, 675];
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
            
            % Set close request fcn to custom function
            this.FigureHandle.CloseRequestFcn = @(~,~)cancelClicked(this);

            this.ParentGridLayout = uigridlayout(this.FigureHandle);
            this.ParentGridLayout.RowHeight = {'fit', 'fit', 'fit', 'fit', '1x', 'fit'};
            this.ParentGridLayout.ColumnWidth = {500};

            addDirSelectors(this, initialDir1, initialDir2);
            
            % Add custom pattern panel
            this.CustomPatternPanel = vision.internal.calibration.webTool.CustomPatternPanel(...
                   this.ParentGridLayout);

            % Add pattern selector options
            this.PatternSelector = vision.internal.calibration.tool.PatternSelector(...
                   this, this.ParentGridLayout, [], vision.getMessage('vision:caltool:DefaultPattern'), true);
            
            % Link pattern selector to custom pattern panel.
            linkPatternSelector(this.CustomPatternPanel, this.PatternSelector)
            
            % Add properties panel
            setupPropertiesPanel(this); 

            % Set default pattern (checkerboard) and related properties
            updateDialog(this, vision.getMessage('vision:caltool:DefaultPattern'));

            this.addOkCancelButton();
            this.UpdateFigureSize();
            this.FigureHandle.ThemeChangedFcn = @(src,evt)this.updateOnThemeChange(src,evt);
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

     methods
        %------------------------------------------------------------------
        function disablePatternProperties(this)
            disable(this.PatternSelector);
            disable(this.CustomPatternPanel);
            
            if isvalid(this.PropertiesPanel)
                panelChildren = findall(this.PropertiesPanel);
                for idx = 1:numel(panelChildren)
                    if ~isa(panelChildren(idx), 'matlab.ui.container.GridLayout') && ...
                            ~isa(panelChildren(idx), 'matlab.ui.container.internal.Accordion') && ...
                            ~isa(panelChildren(idx), 'matlab.ui.container.internal.AccordionPanel')
                        panelChildren(idx).Enable = 'off';
                    end
                end
            end
            
            this.IsFileMode = true;
        end
        
         %------------------------------------------------------------------
        function setupPropertiesPanel(this)
            this.PropertiesPanel = uipanel(this.ParentGridLayout,...
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
                isFiducialTarget = isa(patternDetector, 'vision.calibration.stereo.CharucoBoardDetector') ...
                                || isa(patternDetector, 'vision.calibration.stereo.AprilGridDetector');
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
        function selectCurrentDetector(this)
            selectCurrentDetector(this.PatternSelector);
        end

        %------------------------------------------------------------------
        function updateOnThemeChange(this,~,~)
            if ~isempty(this.PropertiesPanel)
                % Update the checkerboard images
                s = settings;
                patternName =  s.vision.stereoCalibrator.CurrentPattern.ActiveValue;
                updateDialog(this, patternName);
            end
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function addDirSelectors(this, initialDir1, initialDir2)
            import vision.internal.calibration.tool.*;
            
            tagName = 'Camera1';
            this.DirSelector1 = vision.internal.calibration.webTool.DirectorySelector(...
                vision.getMessage('vision:caltool:StereoFolder1Prompt'), ...
                this.ParentGridLayout, initialDir1, tagName);
            set(this.DirSelector1.BrowseButton, 'ButtonPushedFcn', @this.onBrowse1);
            
            tagName = 'Camera2';
            this.DirSelector2 = vision.internal.calibration.webTool.DirectorySelector(...
                vision.getMessage('vision:caltool:StereoFolder2Prompt'), ...
                this.ParentGridLayout, initialDir2, tagName);
            set(this.DirSelector2.BrowseButton, 'ButtonPushedFcn', @this.onBrowse2);
        end

        %------------------------------------------------------------------
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

        %------------------------------------------------------------------
        function UpdateFigureSize(this)
            matlab.ui.internal.PositionUtils.fitToContent(this.FigureHandle, 'topleft');
        end
        
    end
    
    methods(Access=protected)
        %------------------------------------------------------------------
        function onBrowse1(this, ~, ~)
            this.DirSelector1.doBrowse();
            if ~this.DirSelector2.IsModifiedUsingBrowse && ~isempty(this.DirSelector1.SelectedDir)
                parentDir = vision.internal.getParentDir(this.DirSelector1.SelectedDir);
                set(this.DirSelector2.TextBox, 'Value', parentDir);
            end
        end
        
        %------------------------------------------------------------------
        function onBrowse2(this, ~, ~)
            this.DirSelector2.doBrowse();
            if ~this.DirSelector1.IsModifiedUsingBrowse && ~isempty(this.DirSelector2.SelectedDir)
                parentDir = vision.internal.getParentDir(this.DirSelector2.SelectedDir);
                set(this.DirSelector1.TextBox, 'Value', parentDir);
            end
        end
        
        %------------------------------------------------------------------
        function okClicked(this, ~, ~)
            if ~isempty(this.PatternSelector)
                updateUnsavedPatternTemplates(this.PatternSelector);
            end
            
            this.Dir1 = this.DirSelector1.SelectedDir;
            this.Dir2 = this.DirSelector2.SelectedDir;
            
            if areFoldersBad(this)
                return;
            end
            
            fileNames1 = vision.internal.getAllImageFilesFromFolder(this.Dir1);
            fileNames2 = vision.internal.getAllImageFilesFromFolder(this.Dir2);
            
            if areFileNamesBad(this, fileNames1, fileNames2)
                return;
            end
            
            this.FileNames = [fileNames1; fileNames2];
            
            % We want to keep the dialog object alive to remember the
            % pattern choices. Depending on when the dialog needs to be
            % opened, set the visibility to on (add images to existing
            % session) or instantiate a new one (adding images to new
            % session).
            close(this);
            
            if ~isempty(this.CalibratorTool)
                evtData = vision.internal.calibration.tool.StereoPropertiesEventData(this);
                if this.IsFileMode
                    notify(this.CalibratorTool, 'ImageFilesSet', evtData);
                else
                    notify(this.CalibratorTool, 'ImagePropertiesSet', evtData);
                end
            end
        end
        
         %------------------------------------------------------------------
         function cancelClicked(this, ~, ~)
             if ~isempty(this.CalibratorTool)
                 if this.CalibratorTool.Session.hasAnyBoards()
                     % We want to keep the dialog object alive to remember the
                     % pattern choices. Depending on when the dialog needs to be
                     % opened, set the visibility to on (add images to existing
                     % session) or instantiate a new one (adding images to new
                     % session).
                     this.FigureHandle.Visible = 'off';
                 else
                     delete(this.FigureHandle);
                 end
             else
                 delete(this.FigureHandle);
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
        
        %------------------------------------------------------------------
        function tf = areFoldersBad(this)
            errorDlgTitle = vision.getMessage(...
                'vision:caltool:LoadingStereoImagesFailedTitle');
            errorMsg = vision.internal.calibration.tool.checkStereoFolders(...
                this.Dir1, this.Dir2);
            
            if isempty(errorMsg)
                tf = false;
            else
                tf = true;
                uialert(this.FigureHandle, getString(errorMsg), errorDlgTitle);
            end
        end
        
        %------------------------------------------------------------------
        function tf = areFileNamesBad(this, fileNames1, fileNames2)
            errorDlgTitle = vision.getMessage(...
                'vision:caltool:LoadingStereoImagesFailedTitle');
            errorMsg = vision.internal.calibration.tool.checkStereoFileNames(...
                fileNames1, fileNames2, this.Dir1, this.Dir2);
            
            if isempty(errorMsg)
                tf = false;
            else
                tf = true;
                uialert(this.FigureHandle, getString(errorMsg), errorDlgTitle);
            end
        end
    end
end
