classdef ImageCaptureTab < handle
    % ImageCaptureTab Defines key UI elements of the Image Capture Tab of Camera Calibrator App
    %
    %    This class defines all key UI elements and sets up callbacks that
    %    point to methods inside the CameraCalibrationTool class.
    
    % Copyright 2014-2024 The MathWorks, Inc.
    
    %----------------------------------------------------------------------
    % AppContainer Tab
    %----------------------------------------------------------------------
    properties
        Tab
        TabGroup
        Parent
        
        StatusBar
        StatusLabel
    end
    
    properties (Access=private)
        %% Sections
        DeviceSection
        SettingsSection
        CaptureSection
        CloseSection
        
        %% Listeners
        CaptureIntervalSliderListener
        CaptureIntervalEditListener
        SaveLocationEditListener
        NumImagesCaptureEditListener
        NumImagesCaptureSliderListener
        
        % Track button status
        CurrentButtonState
    end
    
    properties (GetAccess=public, SetAccess=private)
        % Current status of Image Capture Tab
        CaptureFlag
        
        % Store device name
        SavedCamera
        
        % All images that image capture tab has acquired in a session
        % (emptied when you close Image Capture Tab).
        Images
        
        % Corresponds to images captured in current Start/Stop
        NumImagesCaptured
        
        % Flag to indicate if any images were captured since tab was open
        AnyImagesCaptured
        
        % Count to keep track of number of patterns in session before
        % image capture
        PrevBoardCount
        
        % Labels for images to show up in the Data Browser
        ImageLabels
        
        % Index where the unsaved images begin.
        StartIndex
        
        % Timer object for capture
        TimerObj
        
        % Timer object for status text
        StatusTimerObj
        TimeRemaining
        
        % Timer object for countdown
    end
    
    properties (Access=private)
        CameraObject = []
        PreviewObject
    end
    
    properties (Constant)
        MinInterval = 1
        MaxInterval = 60
        MinImages = 2
        MaxImages = 100
    end
    
    properties (Access = public)
        PropertiesPanel
    end
    
    events
        % Event for Closing Image Capture Tab.
        CloseTab
        
         % Event for updating toolstrip after adding captured images to session
        UpdateTab
    end
    
    methods (Access=public)
        % Constructor
        function this = ImageCaptureTab(tool)
            % Create Tab group
            this.TabGroup = matlab.ui.internal.toolstrip.TabGroup();
            this.TabGroup.Tag = 'ImageCaptureTabGroup';
            this.TabGroup.Contextual = true;
            
            tabName = vision.getMessage('vision:caltool:ImageCaptureTab');
            
            % Create and Add Image Capture Tab in TabGroup
            this.Tab = matlab.ui.internal.toolstrip.Tab(tabName);
            this.Tab.Tag = strcat(tool.AppContainer.Title,'_',tabName);
            this.TabGroup.add(this.Tab);
            
            % Required for internal purpose
            this.Parent = tool;
            
            this.CaptureFlag = false;
            this.AnyImagesCaptured = false;
            
            % Initialize
            this.reset();
            
            this.createWidgets();
            this.installListeners();
        end
        
        % Implement as inheriting from AbstractTab - do not see use at all
        % now.
        function testers = getTesters(~)
            testers = [];
        end
        
        % Creates and starts a preview.
        function createDevice(this)
            if strcmpi(this.SavedCamera, this.DeviceSection.DeviceComboBox.Value)
                constructWithResolution = true;
            else
                constructWithResolution = false;
                this.PropertiesPanel = [];
            end
            this.updateDeviceSection(this.DeviceSection.DeviceComboBox, constructWithResolution);
        end
        
        % Close preview
        function closePreview(this)
            if ~isempty(this.CameraObject) && isvalid(this.CameraObject)
                closePreview(this.CameraObject); % Stops the timer
            end
        end

        % Delete Camera object
        function deleteCameraObject(this)
            % Delete the Camera Object.
            if (~isempty(this.CameraObject) && isvalid(this.CameraObject) )
                delete(this.CameraObject);
            end
        end

        % Preview
        function preview(this)
            if ~isempty(this.CameraObject) && isvalid(this.CameraObject)
                [width, height] = this.getResolution;
                tool = this.Parent;
                drawImage(tool.ImagePreviewDisplay, width, height);
                replaceImage(tool.ImagePreviewDisplay, width, height);
                preview(this.CameraObject, tool.ImagePreviewDisplay.ImHandle);
            end
        end
        
        % Update property states.
        function updatePropertyStates(this)
            % Disable ability to change devices if images have been
            % captured.
            if this.AnyImagesCaptured
                this.DeviceSection.DeviceComboBox.Enabled = false;
                this.DeviceSection.DeviceComboBox.Description = vision.getMessage('vision:caltool:DisabledCameraDropDownToolTip');
                if ~isempty(this.PropertiesPanel)
                    if isvalid(this.PropertiesPanel.UIfigure)
                        this.PropertiesPanel.DevicePropObjects.Resolution.ComboControl.Enable = false;
                        this.PropertiesPanel.DevicePropObjects.Resolution.ComboControl.Tooltip = vision.getMessage('vision:caltool:DisabledResolutionDropDownToolTip');
                    end
                end
            else
                this.DeviceSection.DeviceComboBox.Enabled = true;
                this.DeviceSection.DeviceComboBox.Description = vision.getMessage('vision:caltool:EnabledCameraDropDownToolTip');
                if ~isempty(this.PropertiesPanel)
                    if isvalid(this.PropertiesPanel.UIfigure)
                        this.PropertiesPanel.DevicePropObjects.Resolution.ComboControl.Enable = true;
                        this.PropertiesPanel.DevicePropObjects.Resolution.ComboControl.Tooltip = vision.getMessage('vision:caltool:EnabledResolutionDropDownToolTip');
                    end
                end
            end
        end
        
        function resetAll(this)
            this.AnyImagesCaptured = false;
            
            % Reset properties panel here.
            this.PropertiesPanel = [];
            this.SavedCamera = [];
            
            reset(this);
        end
        
        function verifyIfPackageInstalled(this, verifySPKGFlag)
            this.enumerateCameras(verifySPKGFlag); % as this is private access method we are accessing from
            % calibrationTool this way
        end
    end
    
    
    methods(Static)
        function tf = hasWritePermissions(~, path, throwError, parent)
            
            tf = false;
            % Create random folder name to try writing.
            [~, tempFolderName] = fileparts(tempname);
            
            dirExists = exist(path, 'dir');
            if (dirExists)
                writable = mkdir(fullfile(path, tempFolderName));
                if writable
                    % Delete the temp dir.
                    rmdir(fullfile(path, tempFolderName));
                    tf = true;
                    return;
                end
                % Error here.
                if throwError
                    uialert(parent,vision.getMessage('vision:caltool:PathWithInvalidPermissionsMsg'), ...
                        vision.getMessage('vision:caltool:PathWithInvalidPermissionsTitle'));
                end
                return;
            end
            
            % Error.
            if throwError
                uialert(parent,vision.getMessage('vision:caltool:InvalidPathMsg'), ...
                    vision.getMessage('vision:caltool:InvalidPathTitle'));
            end
        end
        
    end
    
    
    methods (Access=private)
        
        function createWidgets(this)
            % Creates the widgets on the toolstrip
            
            %% Create Device Widgets
            createDeviceWidgets(this);
            
            %% Create Settings Widgets
            createSettingsWidgets(this);
            
            %% Create Capture Widgets
            createCaptureWidgets(this);
            
            %% Create Close Widgets
            createCloseWidgets(this);
            
            %% Update button states.
            updateButtonStates(this, this.CurrentButtonState);
            
        end
        
        function createDeviceWidgets(this)
            this.DeviceSection = vision.internal.calibration.tool.section.DeviceSection;
            devCB = this.DeviceSection.DeviceComboBox;
            allCams = this.enumerateCameras(false);
            for i = 1:numel(allCams)
                devCB.addItem(allCams{i});
            end
            
            devCB.removeItem(1); % Removing the empty item given as dummy
            % input while creating dropdown button in
            % DeviceSection.
            if (devCB.SelectedIndex == -1)
                devCB.SelectedIndex = 1;  % select 1st item in the dropdown
            end
            
            this.Tab.add(this.DeviceSection.Section);
        end
        
        function createSettingsWidgets(this)
            this.SettingsSection = vision.internal.calibration.tool.section.SettingsSection;
            
            % Add the section to the tab.
            this.Tab.add(this.SettingsSection.Section);
        end
        
        function createCaptureWidgets(this)
            this.CaptureSection = vision.internal.calibration.tool.section.CaptureSection;
            % Add the section to the tab.
            this.Tab.add(this.CaptureSection.Section);
            if isempty(this.SettingsSection.CurrentSaveLocation)
                toolTipID = 'vision:caltool:DisabledStartCaptureButtonToolTip';
            else
                toolTipID = 'vision:caltool:StartCaptureButtonToolTip';
            end
            this.CaptureSection.CaptureButton.Description = vision.getMessage(toolTipID);
            
        end
        
        function createCloseWidgets(this)
            this.CloseSection = vision.internal.calibration.tool.section.CloseSection;
            % Add the section to the tab.
            this.Tab.add(this.CloseSection.Section);
        end
        
        function installListeners(this)
            
            % Device Section
            addlistener(this.DeviceSection.DeviceComboBox,'ValueChanged',@(~,evt)this.updateDeviceSection(evt.Source));
            addlistener(this.DeviceSection.PropertiesButton,'ButtonPushed',@(~,~) this.cameraPropertiesCallback());
            
            % Settings Section
            this.CaptureIntervalSliderListener = addlistener(this.SettingsSection.CaptureIntervalSlider,'ValueChanged',@(hobj,evt)updateCaptureIntervalSlider(this,hobj));
            this.CaptureIntervalEditListener = addlistener(this.SettingsSection.CaptureIntervalEdit,'ValueChanged',@(hobj,evt)updateCaptureIntervalEdit(this,hobj));
            
            this.NumImagesCaptureEditListener = addlistener(this.SettingsSection.NumImagesCaptureEdit,'ValueChanged',@(hobj,evt)updateNumImagesCaptureEdit(this,hobj));
            this.NumImagesCaptureSliderListener = addlistener(this.SettingsSection.NumImagesCaptureSlider,'ValueChanged',@(hobj,evt)updateNumImagesCaptureSlider(this,hobj));
            
            this.SaveLocationEditListener = addlistener(this.SettingsSection.SaveLocationEdit,'ValueChanged',@(hobj, evt)this.updateSaveLocationEditCallback(hobj));
            
            % Browse button
            addlistener(this.SettingsSection.BrowseButton,'ButtonPushed',@(es,ed)browseCallback(this));
            
            % Capture button.
            addlistener(this.CaptureSection.CaptureButton,'ButtonPushed',@(es,ed)capture(this));
            addlistener(this, 'UpdateTab', @(~,~)updateTabAfterCapture(this));
            
            % Add listener for Closing image capture tab.
            addlistener(this.CloseSection.CloseButton,'ButtonPushed',@(es,ed)closeTab(this));
        end
        
    end
    
    
    %% Callback methods
    methods(Access=private)
        
        % Capture button callback
        function capture(this)
            if (~this.hasWritePermissions('',this.SettingsSection.SaveLocationEdit.Value, true, this.Parent.AppContainer))
                return;
            end
            
            % Error for image size inconsistencies.
            parent = this.Parent;
            currentSession = parent.getSession;
            if currentSession.hasAnyBoards
                imInfoBase = imageinfo(currentSession.PatternSet.FullPathNames{1});
                [width, height] = getResolution(this);
                
                % Use abs value since imfinfo could return negative
                % width/height (eg. BMP: https://en.wikipedia.org/wiki/BMP_file_format)
                if (abs(imInfoBase.Width) ~= width) || ...
                        (abs(imInfoBase.Height) ~= height)
                    % issue an error message
                    uialert(this.Parent.AppContainer,vision.getMessage('vision:caltool:imageSizeInconsistent'), ...
                        vision.getMessage('vision:caltool:GenericErrorTitle'));
                    return;
                end
            end
            
            this.CaptureFlag = ~this.CaptureFlag;
            
            % Update the icons
            updateCaptureIcon(this);
            
            % Update toolstrip status.
            updateToolstripStatus(this);
            
            if this.CaptureFlag
                % Mode: Starting a Capture
                this.TimerObj = internal.IntervalTimer(this.SettingsSection.CaptureIntervalSlider.Value);
                addlistener(this.TimerObj, 'Executing', @(src, event)this.getSnapshot(src, event));
                start(this.TimerObj);
                
                % Start the status timer obj.
                this.TimeRemaining = this.SettingsSection.CaptureIntervalSlider.Value;
                this.StatusTimerObj = internal.IntervalTimer(1);
                addlistener(this.StatusTimerObj, 'Executing', @(src, event)this.statusTimerCallback(src, event));
                start(this.StatusTimerObj);
                
                % Update status text.
                this.setStatusText();
            else
                % Mode: Stopping a capture
                % Stop and delete timer object.
                this.stopTimers;
                
                % No images captured, return.
                if (this.NumImagesCaptured==0)
                    this.reset();
                    return;
                end
                
                % Save images to file
                lastFileID = this.getLastFileID;
                fullPathFileNames = cell(1,this.NumImagesCaptured);
                for idx = 1:this.NumImagesCaptured
                    fName = strcat('Image', num2str(lastFileID+idx), '.png');
                    fullPathFileName = fullfile(this.SettingsSection.SaveLocationEdit.Value, fName);
                    % Index into Images array.
                    index = this.StartIndex + idx - 1;
                    imwrite (this.Images(:,:,:,index), fullPathFileName, 'png');
                    % Update the image strip.
                    this.makeLabel(fName, index);
                    
                    % Create cell of full path file names.
                    fullPathFileNames{idx} = fullPathFileName;
                end
                
                % Run detection and add to PatternSet
                parent = this.Parent;
                currentSession = parent.getSession;
                this.PrevBoardCount = 0;
                if currentSession.hasAnyBoards
                    this.PrevBoardCount = currentSession.PatternSet.NumPatterns;
                end
                parent.addImagesFromCameraToSession(fullPathFileNames);
            end
            
        end
        
        function updateTabAfterCapture(this)
            currentSession = this.Parent.getSession;
            
            % Get the new board count.
            currentBoardCount = 0;
            if currentSession.hasAnyBoards
                currentBoardCount = currentSession.PatternSet.NumPatterns;
            end
            numBoardsAdded = currentBoardCount - this.PrevBoardCount;
            if (numBoardsAdded)
                this.AnyImagesCaptured = true;
                this.updatePropertyStates();
            end
            
            % Remove status text.
            this.setStatusText();
            
            % Reset Capture settings.
            this.reset(); 
        end
        
        function setStatusText(this)
            
            %% Set status text
            statusStr{1} = vision.getMessage ('vision:caltool:NumImagesAcquired',...
                num2str(this.NumImagesCaptured), this.SettingsSection.NumImagesCaptureEdit.Value);
            if (this.TimeRemaining == 1)
                statusStr{2} = vision.getMessage ('vision:caltool:NextCaptureCountdownSingular',...
                    num2str(this.TimeRemaining));
            else
                statusStr{2} = vision.getMessage ('vision:caltool:NextCaptureCountdownPlural',...
                    num2str(this.TimeRemaining));
            end
            
            % Combine the two messages.
            statusStr = strjoin(statusStr, '       ');
            
            if this.CaptureFlag
                setStatusText(this.Parent, statusStr);
            else
                setStatusText(this.Parent, "");
            end
        end
        
        function cams = enumerateCameras(this, verifySPKG)
            
            this.CurrentButtonState = false;
            % Find the location of webcam.
            tabGrp =  this.TabGroup;
            if verifySPKG %while initializing the App, verifySPKG variable
                % will be passed as false to avoid throwing an error
                % dialog. #g2134633 
                % Check if webcam spkg is installed
                fullpathToUtility = which('matlab.webcam.internal.Utility');
                if isempty(fullpathToUtility)
                    cams = {vision.getMessage('vision:caltool:NoWebcamsDetected')};
                    
                    uialert(this.Parent.AppContainer,vision.getMessage('vision:caltool:SupportPkgNotInstalledMsg'), ...
                        vision.getMessage('vision:caltool:GenericErrorTitle'));
                    return;
                end
            end
            % Get available webcams
            try
                cams = webcamlist;
                if isempty(cams)
                    cams = {vision.getMessage('vision:caltool:NoWebcamsDetected')};
                    return;
                end
            catch excep
                cams = {vision.getMessage('vision:caltool:NoWebcamsDetected')};
                if ~isempty(tabGrp.SelectedTab)
                    uialert(this.Parent.AppContainer,excep.message, ...
                        vision.getMessage('vision:caltool:GenericErrorTitle'));
                    
                end
                return;
            end
            
            % Cameras available - so enable them.
            
            this.CurrentButtonState = true;
        end
        function getSnapshot(this, ~, evt)
            % Acquire an image and store it.
            imgCount = evt.ExecutionCount;
            
            if isempty(this.Images)
                this.Images(:,:,:,1) = snapshot(this.CameraObject);
            else
                this.Images(:,:,:,end+1) = snapshot(this.CameraObject);
            end
            if isempty(this.StartIndex)
                this.StartIndex = size(this.Images, 4);
            end
            
            this.NumImagesCaptured = this.NumImagesCaptured + 1;
            this.makeLabel; % Updates the require label to ImageLabels.
            
            % Append images to the strip.
            this.appendImagesToStrip(true);
            
            % Update status text.
            this.setStatusText();
            
            % Check if we have acquired completely.
            if (imgCount >= this.SettingsSection.NumImagesCaptureSlider.Value)
                % Call capture to STOP acquisition.
                capture(this);
            end
        end
        
        function statusTimerCallback(this, ~, ~)
            % Initiate countdown.
            this.TimeRemaining = this.TimeRemaining - 1;
            
            % Reset time.
            if (this.TimeRemaining==0)
                % Update status text.
                this.setStatusText();
                this.TimeRemaining = this.SettingsSection.CaptureIntervalSlider.Value;
            end
            
            % Update status text.
            this.setStatusText();
        end
        
        function appendImagesToStrip(this, showLatest)
            % Update the image strip.
            parent = this.Parent;
            
            % Append image with existing board.
            if showLatest
                if ~isempty(parent.BoardThumbnail) && getNumImages(parent.BoardThumbnail) > 0
                    disableContextMenu(parent.BoardThumbnail);
                    setImageCaptureThumbnail(parent.BoardThumbnail, this.Images, this.ImageLabels);
                else
                    parent.configureBoardThumbnail(false);
                    setImageCaptureThumbnail(parent.BoardThumbnail, this.Images, this.ImageLabels);
                end
            end
        end
        
        function fileID = getLastFileID(this)
            fileID = 0;
            out = dir(fullfile(this.SettingsSection.SaveLocationEdit.Value, 'Image*.png'));
            if ~isempty(out)
                [~, fname]= cellfun(@fileparts, {out.name}, 'UniformOutput', false);
                fNum = cellfun(@(s) (str2double(s(6:end))), fname, 'UniformOutput', false);
                fileID = max([fNum{:}]);
            end
        end
        
        function stopTimers(this)
            % Stop timer object.
            if ( ~isempty(this.TimerObj) && isvalid(this.TimerObj) )
                stop(this.TimerObj);
                this.TimerObj = [];
            end
            
            % Stop status timer object.
            if ( ~isempty(this.StatusTimerObj) && isvalid(this.StatusTimerObj) )
                stop(this.StatusTimerObj);
                this.StatusTimerObj = [];
            end
        end
        
        
        function closeTab(this)
            % Reset the images.
            this.reset();
            if ~isempty(this.PropertiesPanel)
                if isempty(this.PropertiesPanel.SavedResolutionValue)
                    this.PropertiesPanel.SavedResolutionValue = this.PropertiesPanel.DevicePropObjects.Resolution.ComboControl.Value;
                end
                if isvalid(this.PropertiesPanel.UIfigure)
                    close(this.PropertiesPanel.UIfigure)
                end
            end
            
            
            
            % Delete the Camera Object.
            if (~isempty(this.CameraObject) && isvalid(this.CameraObject) )
                delete(this.CameraObject);
            end
            
            % Delete timer object.
            this.stopTimers;
            
            % Notify listener of Close operation.
            notify(this,'CloseTab');
        end
        
        function reset(this)
            % Reset the images.
            this.NumImagesCaptured = 0;
            this.Images = uint8([]);
            this.ImageLabels = [];
            this.StartIndex = [];
        end
        
        function browseCallback(this)
            
            % Call to select directory
            path = uigetdir(this.SettingsSection.SaveLocationEdit.Value, vision.getMessage('vision:caltool:FolderOpenDialogTitle'));
            if ~path % No selection was made.
                return;
            end
            hasPermit = this.hasWritePermissions('',path, true, this.Parent.AppContainer);
            if hasPermit
                % Update the path in the text field.
                this.SettingsSection.SaveLocationEdit.Value = path;
                this.SettingsSection.CurrentSaveLocation = this.SettingsSection.SaveLocationEdit.Value;
                this.CaptureSection.CaptureButton.Enabled = true;
                this.CaptureSection.CaptureButton.Description = vision.getMessage('vision:caltool:StartCaptureButtonToolTip');
                return;
            end
        end
        
        function makeLabel(this, varargin)
            if (nargin>1)
                fileName = varargin{1};
                loc = varargin{2};
                [~, fname, ext] = fileparts(fileName);
                label = [fname, ext];
                this.ImageLabels{loc} = label;
            else
                label = 'Not yet saved to disk';
                this.ImageLabels{end+1} = label;
            end
        end
        
        function updateCaptureIntervalEdit(this, obj)
            % Move slider position to otsu level and update text.
            val = floor(str2double(obj.Value));
            
            if isnan(val)
                this.SettingsSection.CaptureIntervalEdit.Value = num2str(this.SettingsSection.CaptureIntervalSlider.Value);
                return;
            end
            
            % Valid value - continue
            if val > this.MaxInterval
                val = this.MaxInterval;
            elseif val < this.MinInterval
                val = this.MinInterval;
            end
            this.SettingsSection.CaptureIntervalEdit.Value = num2str(val);
            this.SettingsSection.CaptureIntervalSlider.Value = val;
        end
        
        function updateCaptureIntervalSlider(this, ~)
            % Update text.
            sliderVal = floor(this.SettingsSection.CaptureIntervalSlider.Value);
            this.SettingsSection.CaptureIntervalEdit.Value = num2str(sliderVal);
            this.SettingsSection.CaptureIntervalSlider.Value = sliderVal;
        end
        
        function updateNumImagesCaptureEdit(this, obj)
            % Min and Max are 1 and 100.
            val = floor(str2double(obj.Value));
            
            if isnan(val)
                % Do we need an unnecessary error message?
                this.SettingsSection.NumImagesCaptureEdit.Value = num2str(this.SettingsSection.NumImagesCaptureSlider.Value);
                return;
            end
            
            % Valid value - Continue
            if val < this.MinImages
                val = this.MinImages;
            elseif val > this.MaxImages
                val = this.MaxImages;
            end
            
            this.SettingsSection.NumImagesCaptureEdit.Value = num2str(val);
            this.SettingsSection.NumImagesCaptureSlider.Value = val;
            
        end
        
        function updateNumImagesCaptureSlider(this, ~)
            % Update text.
            sliderVal = floor(this.SettingsSection.NumImagesCaptureSlider.Value);
            this.SettingsSection.NumImagesCaptureEdit.Value = num2str(sliderVal);
            this.SettingsSection.NumImagesCaptureSlider.Value = sliderVal;
        end
        
        
        function updateSaveLocationEditCallback(this, evt)
            if this.hasWritePermissions('',evt.Value, true, this.Parent.AppContainer)
                this.SettingsSection.SaveLocationEdit.Value = evt.Value;
                this.SettingsSection.CurrentSaveLocation = this.SettingsSection.SaveLocationEdit.Value;
                this.CaptureSection.CaptureButton.Enabled = true;
                this.CaptureSection.CaptureButton.Description = vision.getMessage('vision:caltool:StartCaptureButtonToolTip');
            else
                this.SettingsSection.SaveLocationEdit.Value = this.SettingsSection.CurrentSaveLocation;
            end
        end
        
        function updateDeviceSection(this, devComboBox, varargin)
            % If no device exists, do nothing and return.
            if ismember(devComboBox.Value, {vision.getMessage('vision:caltool:SPPKGNotInstalled'), vision.getMessage('vision:caltool:NoWebcamsDetected')})
                % Empty the properties panel.
                this.PropertiesPanel = [];
                return;
            end
            
            % Create device
            try
                if (~isempty(this.CameraObject) && isvalid(this.CameraObject) )
                    delete(this.CameraObject);
                    this.CameraObject = [];
                    if ~isempty(this.PropertiesPanel)
                        delete(this.PropertiesPanel.UIfigure);
                    end
                    this.PropertiesPanel = [];
                    this.closePreview();
                end
                
                if (nargin==3)
                    useResolution = varargin{1};
                    if useResolution && ~isempty(this.PropertiesPanel)
                        this.CameraObject = webcam(devComboBox.SelectedIndex, 'Resolution', this.PropertiesPanel.SavedResolutionValue);
                    else
                        this.CameraObject = webcam(devComboBox.SelectedIndex);
                    end
                else
                    this.CameraObject = webcam(devComboBox.SelectedIndex);
                end
                this.preview();
                % Save the device.
                this.SavedCamera = devComboBox.Value;
                
                % Disable buttons
                this.updateButtonStates(true);
            catch excep
                % The camera is in use by another application.
                uialert(this.Parent.AppContainer,excep.message, ...
                    vision.getMessage('vision:caltool:CameraInUseTitle'));
                
                % Disable buttons
                this.updateButtonStates(false);
            end
        end
        
        function updateButtonStates(this, flag)
            this.DeviceSection.PropertiesButton.Enabled = flag;
            this.SettingsSection.CaptureIntervalSlider.Enabled = flag;
            this.SettingsSection.CaptureIntervalEdit.Enabled = flag;
            this.SettingsSection.NumImagesCaptureSlider.Enabled = flag;
            this.SettingsSection.NumImagesCaptureEdit.Enabled = flag;
            this.SettingsSection.BrowseButton.Enabled = flag;
            this.SettingsSection.SaveLocationEdit.Enabled = flag;
            this.CaptureSection.CaptureButton.Enabled = flag;
        end
        
        function updateToolstripStatus(this)
            this.DeviceSection.DeviceComboBox.Enabled = ~this.CaptureFlag;
            if ~(isempty(this.PropertiesPanel))
                if isvalid(this.PropertiesPanel.UIfigure)
                    this.PropertiesPanel.SavedResolutionValue = this.PropertiesPanel.DevicePropObjects.Resolution.ComboControl.Value;
                    close(this.PropertiesPanel.UIfigure)
                end
            end
            
            this.SettingsSection.CaptureIntervalSlider.Enabled = ~this.CaptureFlag;
            this.SettingsSection.CaptureIntervalEdit.Enabled = ~this.CaptureFlag;
            this.DeviceSection.PropertiesButton.Enabled = ~this.CaptureFlag;
            this.SettingsSection.BrowseButton.Enabled = ~this.CaptureFlag;
            this.CloseSection.CloseButton.Enabled = ~this.CaptureFlag;
            this.SettingsSection.NumImagesCaptureSlider.Enabled = ~this.CaptureFlag;
            this.SettingsSection.NumImagesCaptureEdit.Enabled = ~this.CaptureFlag;
            this.SettingsSection.SaveLocationEdit.Enabled = ~this.CaptureFlag;
        end
        
        function updateCaptureIcon(this)
            if this.CaptureFlag
                % Currently capture started, hence show STOP icon.
                this.CaptureSection.CaptureButton.Icon = matlab.ui.internal.toolstrip.Icon('stop');
                this.CaptureSection.CaptureButton.Description = vision.getMessage('vision:caltool:StopCaptureButtonToolTip');
            else
                % Currently stop initiated, hence show START icon.
                this.CaptureSection.CaptureButton.Icon = matlab.ui.internal.toolstrip.Icon('playControl');
                this.CaptureSection.CaptureButton.Description = vision.getMessage('vision:caltool:StartCaptureButtonToolTip');
            end
        end
        
        function cameraPropertiesCallback(this)
            this.checkForConfigurableProperties();
          
        end

        function checkForConfigurableProperties(this)
            selectedCameraIndex = this.DeviceSection.DeviceComboBox.SelectedIndex;
            info = propertyInfo(this.CameraObject(selectedCameraIndex));
            editablePropertiesAvailable = ~isempty(find(~[info.ReadOnly], 1));
            if editablePropertiesAvailable

                this.createPropertiesPanel();
                this.updatePropertyStates();
            else
                this.DeviceSection.PropertiesButton.Enabled = false;
            end

        end
        
        function createPropertiesPanel(this)

            tool = this.Parent;
            if isempty(this.PropertiesPanel)|| ~isvalid(this.PropertiesPanel.UIfigure)
                
                this.PropertiesPanel = vision.internal.calibration.tool.CameraPropertiesPanel(this.CameraObject, tool.ImagePreviewDisplay);
            else
                this.PropertiesPanel.updateCameraObject(this.CameraObject, tool.ImagePreviewDisplay);
                figure(this.PropertiesPanel.UIfigure)
            end
        end
        
        function [width, height] = getResolution(this)
            res = this.CameraObject.Resolution;
            idx = strfind(res, 'x');
            width = str2double(res(1:idx-1));
            height = str2double(res(idx+1:end));
        end

    end
end

function info = imageinfo(filenames)
warnstruct = warning('off', 'imageio:tifftagsread:badTagValueDivisionByZero');
onCleanup(@()warning(warnstruct)); %#ok<UNONC>
info = imfinfo(filenames);
end
