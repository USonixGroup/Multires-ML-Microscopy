% CameraCalibrationTool Main class for the Camera Calibrator App
%
%    This object implements the core routines in Camera Calibrator App.
%    All the callbacks that you see in the UI are implemented below.
%
%    NOTES
%    =====
%    1. To invoke the tool, follow these steps:
%       >>  tool = vision.internal.calibration.webTool.CameraCalibrationTool;
%       >>  tool.show();
%       or simply invoke cameraCalibrator.m
%
%    2. To manage tool instances and to be able to close all tools,
%       a persistent variable is used. It is protected by mlock. That means
%       that clear classes will not unload this class unless all UI
%       instances are closed. At that point, the tool calls munlock and a
%       "clear classes" command can actually unload the class from memory.
%       You can always verify if the class is fully unlocked by calling:
%       >> mislocked vision.internal.calibration.webTool.CameraCalibrationTool
%
%    3. Naming conventions
%       a. Main set of callbacks for tool-strip buttons use simple verbs,
%          e.g. calibrate, export, saveSession, etc.
%       b. Smaller callback use the verb "do", e.g. doKeyPress,
%          doEditCallback, etc.
%       c. Methods are camel-cased.

% Copyright 2012-2024 The MathWorks, Inc.

classdef CameraCalibrationTool < handle
    
    properties(Access=private)
        
        % Tool Tab management
        CalibrationTab
        ImageCaptureTab
        
        % Stereo or single camera
        IsStereo = false;
        
        % Handles the MainImage figure and Thumbnail figure
        MainImageDisplay;
        ReprojectionErrorsDisplay;
        ExtrinsicsDisplayCamera;
        ExtrinsicsDisplayPattern;
        ThumbnailDisplay;
        
        MinBoards = 2; % minimum number of boards required for calibration
        
        OpenSessionPath;
        
        % A flag indicating that doSelection() is in progress to prevent
        % doDeleteKey from executing.
        StillDrawing = false;
        
        % In single camera calibrator this is a char array. In stereo camera
        % calibrator this is a cell array of two char arrays.
        LastImageDir = {};
        
        ImagePropertiesDialog;
        
        ImageCaptureFlag;
        
        Files;
    end
    
    properties (Access=public, Hidden)
        % Thumbnail Handle
        BoardThumbnail
        
        ImagePreviewDisplay;
    end
    
    %----------------------------------------------------------------------
    % App-Container
    %----------------------------------------------------------------------
    properties
        % AppContainer hold App instances.
        AppContainer
        
        % Quick Access bar Help
        QABHelp
        
        % Status-Bar Components
        StatusBar
        StatusLabel
        
        % SessionManager object that handles saving/loading of the session
        SessionManager
        
        % Session object that containing the App's data
        Session
        
        % Figure-Panel for thumbnail
        ThumbnailDataBrowser
        
        % Documents Groups accocated with different displays
        CalibrationImageDocGroup
        CalibrationPreviewDocGroup
        CalibrationPatternDisplayDocGroup

        % Light or Dark Theme
        Theme
    end

    %----------------------------------------------------------------------
    % Figure Handles
    %----------------------------------------------------------------------
    properties (Access = private)
        ImageStatsDlg
        ExportDlg
        LoadDlg
        OptimizationOptionDlg
    end
    
    % Events for adding custom pattern detectors and obtaining image files
    % and related properties
    events
        %------------------------------------------------------------------
        % Events used to control the modality of the image properties
        % dialog. This feature is planned for AppContainer based apps and
        % when we transition to a Java-free version of the app, we can
        % leverage this instead of the custom solution used here:
        % https://jira.mathworks.com/browse/GBT-2698
        AppFocusRestored;
        
        AppFocusLost;
        
        %------------------------------------------------------------------
        ImagePropertiesSet;
        
        ImageFilesSet;
        
        ImagePropertiesClosed;
    end
    
    %----------------------------------------------------------------------
    % Public methods
    %----------------------------------------------------------------------
    methods (Access=public)
        
        %------------------------------------------------------------------
        function this = CameraCalibrationTool(isStereo)
            if nargin == 0
                isStereo = false;
            end
            
            this.IsStereo = isStereo;
            
            import vision.internal.calibration.*;
            
            % Assign unique ID to each App instance. This avoids sharing
            % same model across multiple open App
            if isStereo
                app.Title   = vision.getMessage('vision:caltool:StereoToolTitle');
                app.Tag     = "StereoCameraCalibrator" + "_" + matlab.lang.internal.uuid;
                app.Product = "Computer Vision Toolbox";
                app.Scope   = "Stereo Camera Calibrator";
            else
                app.Title   = vision.getMessage('vision:caltool:ToolTitle');
                app.Tag     = "CameraCalibrator" + "_" + matlab.lang.internal.uuid;
                app.Product = "Computer Vision Toolbox";
                app.Scope   = "Camera Calibrator";
            end
            app.EnableTheming = true;
            this.AppContainer = ...
                matlab.ui.container.internal.AppContainer(app);
            
            % Initialize tabs
            this.CalibrationTab = webTool.CalibrationTab(this, isStereo);
            this.AppContainer.add(this.CalibrationTab.TabGroup);
            
            this.ImageCaptureTab = webTool.ImageCaptureTab(this);
            this.AppContainer.add(this.ImageCaptureTab.TabGroup);
            
            this.SessionManager = ...
                vision.internal.calibration.tool.CalibrationSessionManager;
            this.SessionManager.AppName = app.Title;
            this.SessionManager.IsStereo = this.IsStereo;
            
            this.Session = tool.Session; % initialize the session object
            if isStereo
                this.Session.ExportVariableName = 'stereoParams';
            end
            
            this.setDefaultOptions();

            this.setTheme();
            
            % handle closing of the group
            this.AppContainer.CanCloseFcn = @this.closingSession;
           
            wireUpListeners(this);
            
            % manageToolInstances
            this.addToolInstance();
            
            % set the path for opening sessions to the current directory
            this.OpenSessionPath = pwd;
        end
        
        %------------------------------------------------------------------
        function show(this)
            
            this.setupQABhelp();
            % Create default window layout
            % Show app
            this.createDefaultLayout();
            
            % Add status-bar in the App
            this.createStatusBar();
            
            % Display Initial data Browser message in left pan
            this.ThumbnailDisplay.displayInitialDataBrowserMessage(this.IsStereo);
            
            % update all button states to indicate the tool's state
            updateButtonStates(this);
            
            % Show app
            this.AppContainer.Visible = true;
            this.AppContainer.bringToFront();
            
            drawnow();
        end
        
    end % public methods
    
    %----------------------------------------------------------------------
    % Many of the methods below are public because they are used by tests
    % or by CalibrationTab, but they still should not be used outside of
    % these two areas.
    %----------------------------------------------------------------------
    methods (Access=public, Hidden)
        
        %------------------------------------------------------------------
        % Is the App in stereo mode or single camera mode?
        %------------------------------------------------------------------
        function TF = isStereoAppMode(this)
            TF = this.IsStereo;
        end
        
        %------------------------------------------------------------------
        % New session button callback
        %------------------------------------------------------------------
        function newSession(this)
            % First check if we need to save anything before we wipe
            % existing data
            isCanceled = this.processSessionSaving();
            if isCanceled
                return;
            end
            
            % Wipe the UI clean
            this.resetAll;
            
            % Reset the camera model as well
            this.setDefaultOptions();
            
            this.CalibrationTab.enableNumRadialCoefficients();
            
            % Update the button states
            updateButtonStates(this);
        end
        
        %------------------------------------------------------------------
        % Open session button callback
        %------------------------------------------------------------------
        function openSession(this)
            
            % First check if we need to save anything before we wipe
            % existing data
            isCanceled = this.processSessionSaving();
            if isCanceled
                return;
            end
            
            calFilesString = vision.getMessage('vision:caltool:CalibrationSessionFiles');
            allFilesString = vision.getMessage('vision:uitools:AllFiles');
            selectFileTitle = vision.getMessage('vision:uitools:SelectFileTitle');
            
            % Workaround to get modal behavior in linux machines g2126774
            if ~ispc() && ~ismac()
                this.AppContainer.Busy = 1;
            end

            [filename, pathname] = uigetfile( ...
                {'*.mat', [calFilesString,' (*.mat)']; ...
                '*.*', [allFilesString, ' (*.*)']}, ...
                selectFileTitle, this.OpenSessionPath);
            this.AppContainer.Busy = 0;
            
            wasCanceled = isequal(filename,0) || isequal(pathname,0);
            if wasCanceled
                this.AppContainer.bringToFront();
                return;
            end
            
            % preserve the last path for next time
            this.OpenSessionPath = pathname;
            
            % Indicate that this is going to take some time
            this.AppContainer.Busy = 1;
            
            this.processOpenSession(pathname, filename)
            
            this.AppContainer.Busy = 0;
        end
        
        %------------------------------------------------------------------
        % Save session button callback
        %------------------------------------------------------------------
        function success = saveSession(this, fileName)
            
            % If we didn't save the session before, ask for the filename
            if nargin < 2
                if isempty(this.Session.FileName)
                    fileName = vision.internal.uitools.getSessionFilename(...
                        this.SessionManager.DefaultSessionFileName);
                    if isempty(fileName)
                        success = false;
                        return;
                    end
                else
                    fileName = this.Session.FileName;
                end
            end
            if ~this.IsStereo
                this.Session.StandardCameraModelUI =  this.CalibrationTab.StandardOptionsPanel.CameraModel;
            else
                this.Session.StandardCameraModelUI = this.CalibrationTab.StandardOptionsSection.CameraModel;
            end
            
            if ~this.IsStereo
                this.Session.FisheyeCameraModelUI = this.CalibrationTab.FisheyeOptionsPanel.CameraModel;
            else
                this.Session.FisheyeCameraModelUI = [];
            end
            success = this.SessionManager.saveSession(this.Session, fileName, this.AppContainer);
        end
        
        %------------------------------------------------------------------
        function saveSessionAs(this)
            fileName = vision.internal.uitools.getSessionFilename(...
                this.SessionManager.DefaultSessionFileName);
            if ~isempty(fileName)
                this.saveSession(fileName);
            end
        end
        
        %------------------------------------------------------------------
        % Add images/Add images from file button callback
        %------------------------------------------------------------------
        function addImages(this)
            if this.Session.hasAnyBoards()
                if ~this.IsStereo
                    [files, isUserCanceled] = getImageFiles(this);
                    if isUserCanceled
                        return;
                    end
                    this.Files = [this.Files files];
                    addImagesToExistingSession(this, files);
                else % Stereo case (enable the dialog)
                    if isobjvalid(this.ImagePropertiesDialog)
                        delete(this.ImagePropertiesDialog);
                        this.ImagePropertiesDialog = [];
                        
                        createImagePropertiesDialog(this);
                        disablePatternProperties(this.ImagePropertiesDialog);
                    else
                        createImagePropertiesDialog(this);
                    end
                    drawnow();
                end
            else
                if ~this.IsStereo
                    [files, isUserCanceled] = getImageFiles(this);
                    if isUserCanceled
                        return;
                    end
                    this.Files = files;
                    createImagePropertiesDialog(this);
                else % Stereo case (the same dialog is used for file and property selection)
                    createImagePropertiesDialog(this);
                end
                
                % Select the current detector (persistent across MATLAB
                % sessions
                selectCurrentDetector(this.ImagePropertiesDialog);
            end
        end
        
        %------------------------------------------------------------------
        % Add images from camera button callback
        %------------------------------------------------------------------
        function addImagesFromCamera(this)
            
            % Add Camera Preview Display
            configurePreviewDisplay(this);
            
            % Create the device and launch preview.
            this.ImageCaptureTab.createDevice;
            
            this.ImagePreviewDisplay.makeFigureVisible();
            
            % Will throw the errorDialog if supportPackage is not installed
            verifySPKG = true;
            this.ImageCaptureTab.verifyIfPackageInstalled(verifySPKG);
            
            % Disable buttons in calibration tab.
            this.CalibrationTab.updateTabStatus(false);
            
            % Update camera property states.
            this.ImageCaptureTab.updatePropertyStates();
            
            % Select Camera Tab.
            this.AppContainer.SelectedToolstripTab = struct(...
                'tag','Camera Calibrator_Camera',...
                'title', 'Camera');
            
            % Set focus to imageCaptureTab
            this.ImagePreviewDisplay.Showing = true;
            drawnow();
        end
        
        %------------------------------------------------------------------
        % Add images from camera to a session.
        %------------------------------------------------------------------
        function addImagesFromCameraToSession(this, files)
            this.ImageCaptureFlag = true;
            
            if this.Session.hasAnyBoards()
                this.Files = [this.Files files];
                addImagesToExistingSession(this, files);
            else
                this.Files = files;
                createImagePropertiesDialog(this);
            end
        end
        
        %------------------------------------------------------------------
        % Calibrate button callback
        %------------------------------------------------------------------
        function ok = calibrate(this)
            ok = true;
            
            % get camera model options
            this.Session.CameraModel = this.CalibrationTab.CameraModel;
            
            try
                isFisheyeModel = false;
                isFixedIntrinsics = false;
                if ~this.IsStereo
                    isFisheyeModel = this.CalibrationTab.CameraOptionsSection.IsFisheyeSelected;
                else
                    isFixedIntrinsics = this.CalibrationTab.IntrinsicsOptionsSection.IsFixedIntrinsicsSelected;
                end
                imagesUsed = calibrate(this.Session, isFisheyeModel, isFixedIntrinsics, this.AppContainer);
            catch calibEx
                set(this.AppContainer, 'Busy', 0); % reset the cursor
                uialert(this.AppContainer, calibEx.message, ...
                    vision.getMessage('vision:caltool:CalibrationFailedTitle'));
                
                resetCalibration(this.Session);
                updateButtonStates(this);
                
                ok = false; % indicate failure
                return;
            end
            
            % Create the tiled layout for reprojection and extrinsics
            this.createTiledSection();
            
            % check if some images might have been rejected
            if ~all(imagesUsed)
                % this code path is not very likely
                
                % warn the user about image removal
                uialert(this.AppContainer, vision.getMessage('vision:caltool:badBoards',...
                    sum(~imagesUsed)), 'Warning', 'Icon','warning');
                
                removeIndex = find(~imagesUsed);
                
                 this.Session.PatternSet.removePattern(removeIndex);
                
                this.BoardThumbnail.removeSelectedImage(idxMultiselect, ...
                    this.Session.PatternSet.PartialPatterns);
                
                % select first image on the list after board removal
                this.BoardThumbnail.selectItem (1);
            end
            
            % update session state
            updateButtonStates(this);
            updateRemoveContextMenu(this);
            
            % display calibration results
            this.drawPlots();
            
            % redisplay the board; this time with the undistort button
            this.drawBoard();
            
            if getNumImages(this.BoardThumbnail) > 1
                this.BoardThumbnail.selectItem(1);
            end
        end
        
        %------------------------------------------------------------------
        % Export button callback
        %------------------------------------------------------------------
        function export(this)
            
            if this.IsStereo
                paramsPrompt = vision.getMessage(...
                    'vision:caltool:StereoParamsExportPrompt');
            else
                paramsPrompt = vision.getMessage(...
                    'vision:caltool:CameraParamsExportPrompt');
            end
            
            location = getToolCenter(this);
            this.ExportDlg = vision.internal.calibration.webTool.ExportDlg(...
                paramsPrompt, ...
                this.Session.ExportVariableName, ...
                this.Session.ExportErrorsVariableName, ...
                this.Session.ShouldExportErrors, ...
                location);
            
            wait(this.ExportDlg);
            
            if ~this.ExportDlg.Canceled
                assignin('base', this.ExportDlg.ParamsVarName, this.Session.CameraParameters);
                % display the camera parameters at the command prompt
                evalin('base', this.ExportDlg.ParamsVarName);
                
                if this.ExportDlg.ShouldExportErrors
                    assignin('base', this.ExportDlg.ErrorsVarName, ...
                        this.Session.EstimationErrors);
                    evalin('base',  this.ExportDlg.ErrorsVarName);
                end
                
                % remember the current variable name
                this.Session.ExportVariableName = this.ExportDlg.ParamsVarName;
                this.Session.ExportErrorsVariableName = this.ExportDlg.ErrorsVarName;
                this.Session.ShouldExportErrors = this.ExportDlg.ShouldExportErrors;
            end
        end
        
        %------------------------------------------------------------------
        % Layout button callback
        %------------------------------------------------------------------
        function layout(this)
            % Disable App Interaction
            this.AppContainer.Busy = 1;
            
            % record the threshold line level
            if isobjvalid(this.ReprojectionErrorsDisplay)
                [loc,isLine] = getSliderState(this.ReprojectionErrorsDisplay);
            else
                loc = [];
            end
            
            this.createDefaultLayout();
            
            % if we have data, restore plots
            if this.Session.hasAnyBoards
                if this.Session.isCalibrated()
                    % reset the plots to their original state
                    this.Session.ExtrinsicsView = 'CameraCentric';
                    
                    this.drawPlots();
                    
                    % restore the threshold line level
                    if isobjvalid(this.ReprojectionErrorsDisplay) && ~isempty(loc)
                        restoreSliderState(this.ReprojectionErrorsDisplay,loc,isLine);
                    end
                    
                end
                this.drawBoard();
            end
            
            % Zoom buttons are affected if the main image is restored
            updateButtonStates(this);
            drawnow();
            
            % Re-enable App Interaction
            this.AppContainer.Busy = 0;
        end
        
        %------------------------------------------------------------------
        % Setup Quick Access Bar help button
        %------------------------------------------------------------------
        function setupQABhelp(this)
            % Create and Attach callback to quick bar help button.
            this.QABHelp = matlab.ui.internal.toolstrip.qab.QABHelpButton;
            
            % Wire up the Quick Access Bar help button with custom
            % documentation pages.
            this.QABHelp.ButtonPushedFcn = @this.doHelp;
            this.AppContainer.addQabControl(this.QABHelp);
        end
        
        %------------------------------------------------------------------
        % Help button callback
        %------------------------------------------------------------------
        function doHelp(this,~,~)
            if this.IsStereo
                doc('stereoCameraCalibrator');
            else
                doc('cameraCalibrator');
            end
        end
        
        %------------------------------------------------------------------
        % Setup Status Bar
        %------------------------------------------------------------------
        function createStatusBar(this)
            if isempty(this.StatusBar)
                % Initilize status bar
                this.StatusBar = matlab.ui.internal.statusbar.StatusBar();
                this.StatusBar.Tag = "CalibratorStatusBar";
                
                % Create right bottom corner status text
                this.StatusLabel = matlab.ui.internal.statusbar.StatusLabel();
                this.StatusLabel.Tag = "OutputDirectoryStatusLabel";
                this.StatusLabel.Region = "right";
                this.StatusLabel.Text = "";
                
                this.StatusBar.add(this.StatusLabel);
                
                % Add status bar in app container
                this.AppContainer.add(this.StatusBar);
            end
        end
        
        %------------------------------------------------------------------
        % Update Status Test
        %------------------------------------------------------------------
        function setStatusText(this, statusStr)
            this.StatusLabel.Text = statusStr;
        end
        
        %------------------------------------------------------------------
        % Codegen button callback
        %------------------------------------------------------------------
        function generateCode(this)
            
            codeString = generateCode(this.Session);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Output the generated code to the MATLAB editor
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            editorDoc = matlab.desktop.editor.newDocument(codeString);
            editorDoc.smartIndentContents;
        end
        
        %------------------------------------------------------------------
        % This method is used for testing
        %------------------------------------------------------------------
        function setClosingApprovalNeeded(this, in)
            this.ToolGroup.setClosingApprovalNeeded(in);
        end
        
        %------------------------------------------------------------------
        % This callback updates session state when the camera model UI
        % elements are changed
        %------------------------------------------------------------------
        function cameraModelChanged(this, varargin)
            % Give the cameraModelChanged callback enough time to fire
            drawnow;
            
            if nargin > 1
                isFisheyeSelected = varargin{1};
                if isFisheyeSelected
                    this.CalibrationTab.CameraOptionsSection.selectFisheyeModel();
                else
                    this.CalibrationTab.CameraOptionsSection.selectStandardModel();
                end
            end

            %Update the CameraModel in the Session to match the UI.
            if ~this.IsStereo
                this.Session.StandardCameraModelUI = this.CalibrationTab.StandardOptionsPanel.CameraModel;
                this.Session.FisheyeCameraModelUI = this.CalibrationTab.FisheyeOptionsPanel.CameraModel;
                this.Session.IsFisheyeSelected = this.CalibrationTab.CameraOptionsSection.IsFisheyeSelected;
                
                if(this.Session.IsFisheyeSelected)
                    currentModelUI = this.Session.FisheyeCameraModelUI;
                else
                    currentModelUI = this.Session.StandardCameraModelUI;
                end
            else
                this.Session.StandardCameraModelUI = this.CalibrationTab.StandardOptionsSection.CameraModel;
                currentModelUI = this.Session.StandardCameraModelUI;
            end
            if isCalibrated(this.Session) && ...
                    isequal(this.Session.CameraModel, currentModelUI)
                this.Session.CanExport = true;
            else
                this.Session.CanExport = false;
            end
            
            this.Session.IsChanged = true;
            updateButtonStates(this);
        end
        
        %------------------------------------------------------------------
        % This callback reacts to radio button interactions in the
        % Intrinsics section of the toolstrip. It toggles states between
        % Computed intrinsics and Fixed Intrinsics.
        %------------------------------------------------------------------
        function toggleIntrinsicsMode(this)
            % Give the toggleIntrinsicsMode callback enough time to fire
            drawnow;
            drawnow;
            % Update flag in session to match UI.
            fixedSelected = this.CalibrationTab.IntrinsicsOptionsSection.IsFixedIntrinsicsSelected;
            this.Session.IsFixedIntrinsics = fixedSelected;
            % Don't hold state. Need to recalibrate each time we switch.
            this.Session.CameraModel = [];
            this.Session.CanExport = false;
            this.Session.IsChanged = true;
            
            if ~fixedSelected
                this.MainImageDisplay.hideMessagePane();
            else
                this.MainImageDisplay.showMessagePane();
            end
            
            updateButtonStates(this);
        end
        
        %------------------------------------------------------------------
        % This callback reacts to button presses on the load intrinsics
        % button. It opens a dialog to load both the camera intrinsics, and
        % caches it.
        %------------------------------------------------------------------
        function doLoadIntrinsics(this)
            
            set(this.AppContainer, 'Busy', 1)
            resetWaiting = onCleanup(@() set(this.AppContainer,'Busy',0));
            
            intrinsicsVarName1 = this.Session.StereoIntrinsicsVarName1;
            intrinsicsVarName2 = this.Session.StereoIntrinsicsVarName2;
            
            validClassNames = {'cameraIntrinsics', 'cameraParameters'};
            location = getToolCenter(this);
            this.LoadDlg = vision.internal.calibration.webTool.LoadIntrinsicsDlg(...
                validClassNames, intrinsicsVarName1, intrinsicsVarName2, ...
                location);
            wait(this.LoadDlg);
            
            if ~this.LoadDlg.Canceled
                % Read fixed intrinsics into Session.
                loadSucceeded = false;
                try
                    sessionImageSize = this.Session.PatternSet.ImageSize;
                    this.Session.StereoIntrinsicsVarName1 = this.LoadDlg.VarName1;
                    this.Session.StereoIntrinsics1 = ...
                        extractIntrinsics(this.Session.StereoIntrinsicsVarName1, sessionImageSize);
                    this.Session.StereoIntrinsicsVarName2 = this.LoadDlg.VarName2;
                    this.Session.StereoIntrinsics2 = ...
                        extractIntrinsics(this.Session.StereoIntrinsicsVarName2, sessionImageSize);
                    loadSucceeded = true;
                catch loadingEx
                    uialert(this.AppContainer, loadingEx.message, ...
                        vision.getMessage( ...
                        'vision:caltool:IntrinsicsLoadFailureDialogTitle'));
                end
                
                updateButtonStates(this);
                
                if loadSucceeded
                    % Check if the variable names are same: warn if so.
                    if strcmp(this.Session.StereoIntrinsicsVarName1, this.Session.StereoIntrinsicsVarName2)
                        uialert(this.AppContainer, getString(message('vision:caltool:sameStereoIntrinsicsLoaded')),...
                            'Warning', 'Icon','warning');
                    end
                    % Show yellow banner indicating that intrinsics are loaded
                    this.MainImageDisplay.addMessagePane(vision.getMessage( ...
                        'vision:caltool:SuccessLoadingIntrinsics', this.Session.StereoIntrinsicsVarName1, this.Session.StereoIntrinsicsVarName2));
                end
            end
            
            %--------------------------------------------------------------
            function obj = extractIntrinsics(varName, sessionImageSize)
                % Create 'cameraIntrinsics' object from name of base
                % workspace variable. If the object is a 'cameraParameters'
                % object, construct the 'cameraIntrinsics' object from its
                % parameters.
                
                obj = evalin('base', varName);
                if isa(obj, 'cameraParameters')
                    thisImageSize = obj.ImageSize;
                    if isempty(obj.ImageSize)
                        thisImageSize = sessionImageSize;
                    end
                    obj = cameraIntrinsics(obj.FocalLength, ...
                        obj.PrincipalPoint, ...
                        thisImageSize, ...
                        'RadialDistortion', obj.RadialDistortion,...
                        'TangentialDistortion',obj.TangentialDistortion ,...
                        'Skew', obj.Skew);
                end
            end
        end
        
        %------------------------------------------------------------------
        % This callback reacts to the 'Show Undistorted' button on the
        % toolstrip and changes the document area to reflect either
        % original, or undistorted image.
        %------------------------------------------------------------------
        function imageViewChanged(this)
            % Change the document area to show the undistorted image
            % Change the button text to toggle between 'show undistorted'
            % and 'show original'.
            boardIdx = selectedItem(this.BoardThumbnail);
            
            % Enter this block only if valid boardIdx and is calibrated
            if boardIdx > 0 && this.Session.isCalibrated()
                board = this.Session.PatternSet.getPattern(boardIdx);
                cameraParams = this.Session.CameraParameters;
                
                isOriginal = this.CalibrationTab.ViewUndistortSection.isUndistortButtonPressed();
                if isOriginal
                    scaleFactor = this.CalibrationTab.ViewUndistortSection.getScaleFactorValue();
                    this.MainImageDisplay.drawUndistortedBoard(board, ...
                        cameraParams, scaleFactor);
                    toolTipID = 'vision:caltool:UndistortOriginalToolTip';
                    this.CalibrationTab.ViewUndistortSection.setUndistortButtonToolTip(toolTipID);
                    this.CalibrationTab.ViewUndistortSection.setUndistortButtonIcon('distorted');
                    if this.Session.IsFisheyeModel
                        this.CalibrationTab.ViewUndistortSection.updateScaleFactorControls(true);
                    end
                else
                    this.MainImageDisplay.drawBoard(board, boardIdx, ...
                        cameraParams);
                    toolTipID = 'vision:caltool:UndistortUndistortedToolTip';
                    this.CalibrationTab.ViewUndistortSection.setUndistortButtonToolTip(toolTipID);
                    this.CalibrationTab.ViewUndistortSection.setUndistortButtonIcon('original');
                    this.CalibrationTab.ViewUndistortSection.updateScaleFactorControls(false);
                end
            end
        end
        
        %------------------------------------------------------------------
        % This callback reacts to the apply button in the scale factor
        % spinner. It applies the specified scale factor, and
        % updates the document area with the new image.
        %------------------------------------------------------------------
        function scaleFactorApplied(this)
            if  this.CalibrationTab.ViewUndistortSection.isUndistortButtonPressed()
                this.CalibrationTab.ViewUndistortSection.updateAppliedScaleFactor();
                this.drawBoard();
            end
        end
        
        %------------------------------------------------------------------
        % This callback reacts to clicking the 'Show Rectified' toggle button.
        % It updates the document area with the rectified images and switches
        % the button to be pressed-down. Pressing it again will revert the action.
        %------------------------------------------------------------------
        function rectifiedViewChanged(this)
            
            boardIdx = selectedItem(this.BoardThumbnail);
            
            % Enter this block only if valid boardIdx and is calibrated
            if boardIdx > 0 && this.Session.isCalibrated()
                board = this.Session.PatternSet.getPattern(boardIdx);
                cameraParams = this.Session.CameraParameters;
                
                isOriginal = this.CalibrationTab.ViewUndistortSection.isButtonPressed();
                if isOriginal
                    this.MainImageDisplay.drawRectifiedImages(board, cameraParams);
                    toolTipId = 'vision:caltool:ShowOriginalStereoImagesToolTip';
                    this.CalibrationTab.ViewUndistortSection.setButtonToolTip(toolTipId);
                else
                    this.MainImageDisplay.drawOriginalImages(board, boardIdx, cameraParams);
                    toolTipId = 'vision:caltool:ShowRectifiedStereoImagesToolTip';
                    this.CalibrationTab.ViewUndistortSection.setButtonToolTip(toolTipId);
                end
            end
        end
        
        %------------------------------------------------------------------
        function popup = doCameraOptions(this)
            
            this.CalibrationTab.OptionsPopup  = this.CalibrationTab.CameraOptionsSection.getOptionButton();
            if this.CalibrationTab.CameraOptionsSection.IsFisheyeSelected
                this.CalibrationTab.OptionsPopup.Popup = this.CalibrationTab.FisheyeOptionsPanel.Panel;
            else
                this.CalibrationTab.OptionsPopup.Popup = this.CalibrationTab.StandardOptionsPanel.PopupList;
            end
            
            this.CalibrationTab.CameraOptionsSection.OptionButton.Popup = this.CalibrationTab.OptionsPopup.Popup;
            popup  = this.CalibrationTab.OptionsPopup.Popup;
        end
        
        %------------------------------------------------------------------
        function doInitializationOptions(this)
            
            location = getToolCenter(this);
            this.OptimizationOptionDlg = vision.internal.calibration.webTool.OptimizationOptionsDlg(...
                this.Session.OptimizationOptions, location);
            wait(this.OptimizationOptionDlg);
            
            if isempty(this.Session.OptimizationOptions)
                this.Session.OptimizationOptions.InitialIntrinsics = [];
                this.Session.OptimizationOptions.InitialDistortion = [];
            end
            
            if ~isequal(this.OptimizationOptionDlg.OptimizationOptions, this.Session.OptimizationOptions)
                this.Session.OptimizationOptions = this.OptimizationOptionDlg.OptimizationOptions;
                
                if isempty(this.Session.OptimizationOptions.InitialDistortion)
                    this.CalibrationTab.enableNumRadialCoefficients();
                else
                    numCoeffs = numel(this.Session.OptimizationOptions.InitialDistortion);
                    this.Session.CameraModel.NumDistortionCoefficients = numCoeffs;

                    % Remove Standard Options Panel Listener to maintain
                    % synchronize callback. 
                    % The cameraModelChange() callback is triggering
                    % asynchronously after setting Standard Options Panel
                    % which is overwriting updateButtonStates() callback
                    % and causing unexpected behavior.
                    this.CalibrationTab.removeRadialCoeffsButtonListener();

                    this.CalibrationTab.disableNumRadialCoefficients(numCoeffs);
                    cameraModelChanged(this);

                    % Restore Standard Options Panel Listener 
                    this.CalibrationTab.installRadialCoeffsButtonListener();
                end
                
                % This drawnow is necessary. Otherwise there is a timing
                % issue that prevents the session flags from being set
                % correctly.
                % The drawnow is replaced with pause as workaround which
                % able to handle Standard Options Panel synchronize
                % callback along with session flag
                pause(0.1);
                this.Session.IsChanged = true;
                this.Session.CanExport = false;
            end
            
            updateButtonStates(this);
        end
        
        %------------------------------------------------------------------
        function deleteToolInstance(this, toolManageFlag)
            this.manageToolInstances('delete', this, toolManageFlag);
        end
        
        %------------------------------------------------------------------
        function processOpenSession(this, pathname, filename)
            this.resetAll();  % Start fresh
            
            session = this.SessionManager.loadSession(pathname, filename, this.AppContainer);
            if isempty(session)
                return;
            end
            
            this.Session = session;
            
            % Restore the state of the buttons related to the camera model
            % Restore state of camera model: standard vs. fisheye.
            this.restoreCameraModel();
            
            if ~isempty(this.Session.PatternSet)
                % Proceed only if the BoardsSet was initialized at least
                % once; even if it doesn't hold any boards
                
                % In 17a we switched from short unit names to full unit names
                % to better support translation of the tool.  Substitute new strings
                % upon loading of an old session file.
                switch this.Session.PatternSet.Units
                    case {'mm'}
                        this.Session.PatternSet.Units = 'millimeters';
                    case {'cm'}
                        this.Session.PatternSet.Units = 'centimeters';
                    case {'in'}
                        this.Session.PatternSet.Units = 'inches';
                end
                
                % PartialBoards was added in 21a to support partial
                % checkerboards in camera calibration
                 if isempty(this.Session.PatternSet.PartialPatterns)
                    this.Session.PatternSet.PartialPatterns = false(this.Session.PatternSet.NumPatterns, 1);
                 end
                
                % Restore image strip
                configureBoardThumbnail(this, true);
                setBrowserContent(this.BoardThumbnail,  this.Session.PatternSet, false);
                
                % create tiles if needed
                if this.Session.isCalibrated()
                    this.createTiledSection();
                    this.drawPlots();  % Restore calibration plots if available
                end
                this.drawBoard(); % Display first board on the list
                
                selectItem(this.BoardThumbnail, 1);

                % Preserve the CanExport flag before setting the options
                % which triggers a callback that resets the flag
                keepExportFlag = this.Session.CanExport;
                
                % Restore the CanExport flag
                this.Session.CanExport = keepExportFlag;
                
                % Update main UI buttons
                updateButtonStates(this);
                updateRemoveContextMenu(this);
            end
            
            if this.Session.IsFixedIntrinsics
                % Check if intrinsics are loaded, and if they exist in
                % workspace. If loaded but not in workspace, warn.
                if ~isempty(this.Session.StereoIntrinsics1) && ...
                        ~isempty(this.Session.StereoIntrinsics2)
                    checkWorkspaceVariables(this);
                end
            else
                if isempty(this.Session.OptimizationOptions) || ...
                        isempty(this.Session.OptimizationOptions.InitialDistortion)
                    this.CalibrationTab.enableNumRadialCoefficients();
                else
                    this.CalibrationTab.disableNumRadialCoefficients(...
                        numel(this.Session.OptimizationOptions.InitialDistortion));
                end
            end
            
            this.Session.IsChanged = false; % we just loaded it now
        end
        
        %------------------------------------------------------------------
        function TF = hasLoadedImages(this)
            TF = false;
            try
                TF = (this.Session.PatternSet.NumPatterns > 0);
            catch
                % Do nothing. TF will remain false
            end
        end
        
    end %public hidden
    
    %----------------------------------------------------------------------
    % Private methods
    %----------------------------------------------------------------------
    methods (Access=private)
        %------------------------------------------------------------------
        function setDefaultOptions(this)
            numRadialCoeffs       = 2;
            computeSkew           = false;
            computeTangentialDist = false;
            isFisheyeModel        = false;
            estimateAlignment     = false;
            isStereo              = this.IsStereo;
            
            % Make options section reflect default options.
            this.CalibrationTab.setCameraModelOptions(isStereo, numRadialCoeffs, ...
                computeSkew, computeTangentialDist, isFisheyeModel, ...
                estimateAlignment);
            
            if isStereo
                this.CalibrationTab.setDefaultIntrinsicsOptions();
            end
            drawnow;
            drawnow; % lets the button callback fire before we change
            % the session state below
            
            this.Session.IsChanged = false;
        end
        
        %------------------------------------------------------------------
        function restoreCameraModel(this)
            
            % Restore the calibration configuration button states
            if isfield(this.Session.StandardCameraModelUI, 'ComputeSkew')
                computeSkew = this.Session.StandardCameraModelUI.ComputeSkew;
            else
                computeSkew = false; % set to default.
            end
            
            if isfield(this.Session.StandardCameraModelUI, 'ComputeTangentialDistortion')
                computeTangentialDist = ...
                    this.Session.StandardCameraModelUI.ComputeTangentialDistortion;
            else
                computeTangentialDist = false; % set to default.
            end
            
            if isfield(this.Session.StandardCameraModelUI, 'NumDistortionCoefficients')
                numRadialCoeffs = ...
                    this.Session.StandardCameraModelUI.NumDistortionCoefficients;
            else
                numRadialCoeffs = 2; % set to default.
            end
            
            if isfield(this.Session.FisheyeCameraModelUI, 'EstimateAlignment')
                estimateAlignment = ...
                    this.Session.FisheyeCameraModelUI.EstimateAlignment;
            else
                estimateAlignment = false;
            end
            
            isFisheyeSelected = this.Session.IsFisheyeSelected;
            
            this.CalibrationTab.setCameraModelOptions(this.IsStereo, numRadialCoeffs, ...
                computeSkew, computeTangentialDist, isFisheyeSelected, ...
                estimateAlignment);
            
            drawnow;
            updateButtonStates(this);
            
            % Give the callback enough time to fire
            drawnow;
        end
        
        %------------------------------------------------------------------
        function checkWorkspaceVariables(this)
            % Session must be loaded at this point.
            wkspaceVars = evalin('base', 'whos');
            
            foundIntrinsics1 = false;
            foundIntrinsics2 = false;
            
            for idx = 1:numel(wkspaceVars)
                if strcmp(wkspaceVars(idx).name, this.Session.StereoIntrinsicsVarName1)
                    foundIntrinsics1 = true;
                    break;
                end
            end
            
            for idx = 1:numel(wkspaceVars)
                if strcmp(wkspaceVars(idx).name, this.Session.StereoIntrinsicsVarName2)
                    foundIntrinsics2 = true;
                    break;
                end
            end
            
            if ~foundIntrinsics1 || ~foundIntrinsics2
                this.MainImageDisplay.addMessagePane(vision.getMessage( ...
                    'vision:caltool:SuccessLoadingIntrinsicsFromSession', this.Session.StereoIntrinsicsVarName1, this.Session.StereoIntrinsicsVarName2));
            end
        end
        
        %------------------------------------------------------------------
        function doZoom(this, src, ~) % (src, evnt)
            drawnow();
            
            if ~this.MainImageDisplay.isAxesValid()
                return;
            end
            
            this.MainImageDisplay.makeHandleVisible();
            % remove the listeners while we manipulate button
            % selections
            this.removeZoomListeners();
            drawnow();
            
            switch (src.Tag)
                case 'btnZoomIn'
                    state = this.CalibrationTab.ZoomSection.ZoomInButtonState;
                    this.MainImageDisplay.setZoomInState(state);
                    this.CalibrationTab.ZoomSection.resetButtons();
                    drawnow();
                    this.CalibrationTab.ZoomSection.ZoomInButtonState = state;
                    
                case 'btnZoomOut'
                    state = this.CalibrationTab.ZoomSection.ZoomOutButtonState;
                    this.MainImageDisplay.setZoomOutState(state);
                    this.CalibrationTab.ZoomSection.resetButtons();
                    drawnow();
                    this.CalibrationTab.ZoomSection.ZoomOutButtonState = state;
                    
                case 'btnPan'
                    state = this.CalibrationTab.ZoomSection.PanButtonState;
                    this.MainImageDisplay.setPanState(state);
                    this.CalibrationTab.ZoomSection.resetButtons();
                    drawnow();
                    this.CalibrationTab.ZoomSection.PanButtonState = state;
                    
            end
            
            % let the button selections re-draw
            drawnow();
            
            % add back the listeners
            this.addZoomListeners();
            this.MainImageDisplay.makeHandleInvisible();
            
        end % doZoom
        
        %--------------------------------------------------------------
        function addZoomListeners(this)
            this.CalibrationTab.ZoomSection.addListeners(@this.doZoom);
        end
        
        %--------------------------------------------------------------
        function removeZoomListeners(this)
            
            if ~isempty(this.Session.PatternSet) && (this.Session.PatternSet.NumPatterns > 0)
                this.CalibrationTab.ZoomSection.removeListeners();
                drawnow();
            end
        end
        
        %------------------------------------------------------------------
        function isCanceled = processSessionSaving(this)
            
            isCanceled = false;
            
            sessionChanged = this.Session.IsChanged;
            
            yes    = vision.getMessage('MATLAB:uistring:popupdialogs:Yes');
            no     = vision.getMessage('MATLAB:uistring:popupdialogs:No');
            cancel = vision.getMessage('MATLAB:uistring:popupdialogs:Cancel');
            
            if sessionChanged
                selection = this.askForSavingOfSession(this.AppContainer);
            else
                selection = no;
            end
            
            switch selection
                case yes
                    this.saveSession();
                case no
                    
                case cancel
                    isCanceled = true;
            end
        end
        
        %------------------------------------------------------------------
        function [files, isUserCanceled] = getImageFiles(this)
            
            if isempty(this.LastImageDir) || ~exist(this.LastImageDir, 'dir')
                this.LastImageDir = pwd();
            end
            
            % Workaround to get modal behavior in linux machines g2126774
            if ~ispc() && ~ismac()
                this.AppContainer.Busy = 1;
            end

            [files, isUserCanceled] = imgetfile('MultiSelect', true, ...
                'InitialPath', this.LastImageDir);
            this.AppContainer.Busy = 0;

            % Workaround to avoid window's minimization
            this.AppContainer.bringToFront();

            if ~isUserCanceled && ~isempty(files)
                this.LastImageDir = fileparts(files{1});
            end
        end
        
        %------------------------------------------------------------------
        function closeImageCaptureTab(this)
            if ~isempty(this.ImageCaptureTab.PropertiesPanel)
                if isvalid((this.ImageCaptureTab.PropertiesPanel))
                    delete((this.ImageCaptureTab.PropertiesPanel.UIfigure));
                end
            end
            % Close the preview.
            closePreview(this.ImageCaptureTab);
            
            % Close ImagePreviewDisplay
            this.ImagePreviewDisplay.Opened = false;
            
            % Update button status
            this.CalibrationTab.updateTabStatus(true);
            updateButtonStates(this);
        end
        
        %------------------------------------------------------------------
        function status = isLiveImageCaptureOpen(this)
            % Do not honor this if live capture is running.
            
            status = false;
            
            % If capture tab is open return true.
            if (~this.IsStereo && isImageCaptureTabInGroup(this))
                status = true;
            end
        end
        
    end %private methods
    
    methods(Access=public, Hidden)
        %------------------------------------------------------------------
        function  imageStats = addImagesToNewSession(this,...
                 files, detector, detectorFile)
            
            import vision.internal.calibration.*;
            try
                % Construct pattern set object 
                if isa(detector, 'vision.internal.calibration.monocular.CheckerboardDetector') || ...
                    isa(detector, 'vision.internal.calibration.stereo.CheckerboardDetector') || ...
                    isa(detector, 'vision.calibration.monocular.CheckerboardDetector') || ...
                    isa(detector, 'vision.calibration.stereo.CheckerboardDetector')
                    this.Session.PatternSet = tool.BoardSet(files, detector, detectorFile, this.AppContainer);
                else
                    this.Session.PatternSet = tool.CustomPatternSet(files, detector, detectorFile);
                end
                
                configureBoardThumbnail(this, true);
                
                % Let the user know how many boards were detected if some
                % were missed and how many boards were partially detected.
                imageStats.numProcessed  = size(files, 2);
                imageStats.numAdded      = this.Session.PatternSet.NumPatterns;
                imageStats.numDuplicates = 0;
                imageStats.numPartial = nnz(this.Session.PatternSet.PartialPatterns);
                
                showAddImageStatsDlg(this, imageStats);
                updateAfterAddingImages(this);
            catch loadingEx
                if ~isvalid(this)
                    % we already went through delete sequence; this can
                    % happen if the images did not yet load and someone
                    % already closed the tool
                    return;
                end
                
                if ~isempty(this.ImagePropertiesDialog)
                    this.ImagePropertiesDialog.close();
                end
                
                if ~isempty(this.BoardThumbnail) && getNumImages(this.BoardThumbnail) > 0
                    configureBoardThumbnail(this, false);
                end
                
                this.AppContainer.Busy = 0; % reset the cursor
                uialert(this.AppContainer, loadingEx.message, ...
                    vision.getMessage('vision:caltool:LoadingBoardsFailedTitle'));
                
                % Manage the image strip
                if this.Session.hasAnyBoards()
                    setBrowserContent(this.BoardThumbnail,  this.Session.PatternSet);
                else
                    this.ThumbnailDisplay.displayInitialDataBrowserMessage(this.IsStereo);
                end
            end
        end
        
        function configureBoardThumbnail(this, contextMenuFlag)
            if ~isempty(this.BoardThumbnail)
                deleteBrowser(this.BoardThumbnail);
                this.ThumbnailDisplay.wipeFigure();
            end
            
            this.ThumbnailDisplay.removeDisplayPanel();

                this.BoardThumbnail = vision.internal.calibration.webTool.BoardThumbnailBrowser(...
                    this.ThumbnailDisplay);
            
            if contextMenuFlag
                this.BoardThumbnail.installRemoveContextMenu();
            end
            
            % Thumbnail selection and Key press callback
            this.BoardThumbnail.SelectionCallback = @this.doSelection;
            this.BoardThumbnail.SelectionRemoveCallback = @this.doDeleteKey;
            this.BoardThumbnail.SelectionRemoveAndRecalibrateCallback = @this.removeAndRecalibrate;
            addSelectionListener(this.BoardThumbnail);

            drawnow();
        end
    end %public hidden
    
    methods(Access = private)
        %------------------------------------------------------------------
        % Update Contextmenu
        %------------------------------------------------------------------
        function updateRemoveContextMenu(this)
            if this.Session.isCalibrated() && this.canRecalibrate()
                updateContextMenuMessage(this.BoardThumbnail, ...
                    vision.getMessage('vision:caltool:RemoveAndRecalibrate'));
            else
                updateContextMenuMessage(this.BoardThumbnail, ...
                    vision.getMessage('vision:uitools:Remove'));
            end
        end
        
        %------------------------------------------------------------------
        function updateAfterAddingImages(this)
            % Manage the image strip
            setBrowserContent(this.BoardThumbnail,  this.Session.PatternSet);
            
            installRemoveContextMenu(this.BoardThumbnail);
            
            % Update status text based on thumbnail selection.
            selection = selectedItem(this.BoardThumbnail);

            if this.Session.PatternSet.PartialPatterns(selection)
                statusMsg = vision.getMessage('vision:caltool:PartialPatternStatus');
            else
                statusMsg = vision.getMessage('vision:caltool:CompletePatternStatus');
            end
            this.setStatusText(statusMsg);
            
            % Update session state
            this.Session.CanExport = false;
            this.Session.IsChanged = true;
            if ~this.isLiveImageCaptureOpen
                updateButtonStates(this);
            end
            
            % Update displays
            this.updatePlots();
            this.drawBoard();
        end
        
        %------------------------------------------------------------------
        function imageStats = addImagesToExistingSession(this, files)
            try
                % We are adding to an existing board set
                previousNumBoards = this.Session.PatternSet.NumPatterns;
                imageStats.numDuplicates = this.Session.PatternSet.addPatterns(files, this.AppContainer);
                imageStats.numAdded  = this.Session.PatternSet.NumPatterns - ...
                    previousNumBoards;
                imageStats.numProcessed = size(files, 2);
                
                partialBoardIdx = this.Session.PatternSet.PartialPatterns(previousNumBoards + 1:end);
                imageStats.numPartial = nnz(partialBoardIdx);
                
                showAddImageStatsDlg(this, imageStats);
                updateAfterAddingImages(this);
                
                if this.ImageCaptureFlag
                    this.ImageCaptureFlag = false;
                    notify(this.ImageCaptureTab, 'UpdateTab');
                end
            catch loadingEx
                
                if ~isvalid(this)
                    % we already went through delete sequence; this can
                    % happen if the images did not yet load and someone
                    % already closed the tool
                    return;
                end
                
                uialert(this.AppContainer, loadingEx.message, ...
                    vision.getMessage('vision:caltool:LoadingBoardsFailedTitle'));
                % Manage the image strip
                if this.Session.hasAnyBoards()
                    setBrowserContent(this.BoardThumbnail,  this.Session.PatternSet);
                else
                    this.ThumbnailDisplay.displayInitialDataBrowserMessage(this.IsStereo);
                end
            end
        end
        
        %------------------------------------------------------------------
        function showAddImageStatsDlg(this, imageStats)
            if (imageStats.numAdded == imageStats.numProcessed) && (imageStats.numPartial == 0)
                % nothing to display
                return;
            end
            
            rejectedFileNames = this.Session.PatternSet.LastNonDetectedPathNames;
            
            location = getToolCenter(this);
            if this.IsStereo
                this.ImageStatsDlg = ...
                    vision.internal.calibration.webTool.AddImageStatsStereoDlg(...
                    imageStats, rejectedFileNames, location);
            else
                this.ImageStatsDlg = vision.internal.calibration.webTool.AddImageStatsDlg(...
                    imageStats, rejectedFileNames, location);
            end
            wait(this.ImageStatsDlg);
        end
        
        %------------------------------------------------------------------
        function drawBoard(this)
            % What if the figure has been closed?
            if ~ishandle(this.MainImageDisplay.Figure)
                return;
            end
            
            patternIdx = selectedItem(this.BoardThumbnail);
            
            if patternIdx > 0
                board = this.Session.PatternSet.getPattern(patternIdx);
                cameraParams = this.Session.CameraParameters;
                if ~this.IsStereo
                    % reset the undistort button for a new board.
                    if this.CalibrationTab.ViewUndistortSection.isUndistortButtonPressed()
                        scaleFactor = this.CalibrationTab.ViewUndistortSection.getScaleFactorValue();
                        this.MainImageDisplay.drawUndistortedBoard(board, ...
                            cameraParams, scaleFactor);
                    else
                        this.MainImageDisplay.drawBoard(board, patternIdx, ...
                            cameraParams);
                    end
                else % for the stereo App.
                    if this.CalibrationTab.ViewUndistortSection.isButtonPressed()
                        this.MainImageDisplay.drawRectifiedImages(board, ...
                            cameraParams);
                    else
                        this.MainImageDisplay.drawBoard(board, patternIdx, ...
                            cameraParams);
                    end
                end
            end
        end

        %------------------------------------------------------------------
        function clearBoards(this)

            % delete method required to delete the CoalesceTimer of Board
            % thumbnail
            if ~isempty(this.BoardThumbnail)
                deleteBrowser(this.BoardThumbnail);
                removeBrowserCallbacks(this.BoardThumbnail);
                this.BoardThumbnail = [];
            end

            % wipe the displayed boards in Thumbnail section
            this.ThumbnailDisplay.wipeFigure();
        end

        %------------------------------------------------------------------
        % Puts the image strip in focus
        %------------------------------------------------------------------
        function setFocusOnBoards(this)
            drawnow;
            
            % give focus to the Thumbnail Display to help keyboard access.
            this.ThumbnailDataBrowser.Selected = 1;
            figure(this.ThumbnailDataBrowser.Figure);
        end
        
        %------------------------------------------------------------------
        % Returns true if recalibration is a valid option
        %------------------------------------------------------------------
        function ret = canRecalibrate(this)
            
            idxMultiselect = selectedItem(this.BoardThumbnail);
            
            ret = (this.Session.PatternSet.NumPatterns - ...
                length(idxMultiselect)) >= this.MinBoards;
            
            ret = this.Session.isCalibrated() && ret;
        end
        
        %------------------------------------------------------------------
        function wireUpListeners(this)  
            % While the app is opened the ImageCaptureTab should be hidden
            addlistener(this.ImageCaptureTab, 'CloseTab', @(~,~)closeImageCaptureTab(this));
           
            % Handle action callback for the group
            addlistener(this.AppContainer, 'StateChanged', ...
                @(es,ed)doActionCallback(this, es, ed));

            % Handle callback from image properties dialog (on OK)
            addlistener(this, 'ImagePropertiesSet', @(~,evt) doAddImagesToNewSession(this, evt));
            
            % Handle callback from stereo image properties dialog
            addlistener(this, 'ImageFilesSet', @(~,evt) doAddImagesToExistingSession(this, evt));
            
            % Handle closing of the image properties dialog
            addlistener(this, 'ImagePropertiesClosed', @(~,~) doImagePropertiesClosed(this));
        end
    end
    
    %----------------------------------------------------------------------
    % Smaller Toolstrip Button Callbacks
    %----------------------------------------------------------------------
    methods(Access=public)
        %----------------------------------------------------------
        % Note: the recalibration is done only if there was a prior
        % valid calibration done.  If the boards were only loaded
        % without hitting "calibrate" button, the re-calibration is
        % not invoked.
        %----------------------------------------------------------
        function removeAndRecalibrate(this)
            % Figure out the index list in the case of multi-select
            idxMultiselect = selectedItem(this.BoardThumbnail);
            
            this.processRemoveAndRecalibrate(idxMultiselect)
        end %removeAndRecalibrate
        
        %--------------------------------------------------------------
        function doDeleteKey(this, ~, data)

            % Skip deletion operation during live capture stage
            if isImageCaptureTabInGroup(this)
                return
            end

            % Return if doSelection() is in progress. Otherwise
            % the session may become inconsistent, causing an error.
            if isprop(data, 'Key')
                if this.StillDrawing || ~strcmp(data.Key, 'delete')
                    return;
                end
            end
            
            % If we are in a session with a valid calibration data
            % ask the user if they want to recalibrate and give them
            % an option to bail out; otherwise, don't bother and
            % simply delete the boards
            if this.canRecalibrate()
                question = vision.getMessage('vision:caltool:ConfirmRecalibration');
                title = vision.getMessage('vision:caltool:RecalibrateTitle');
                yes = vision.getMessage('MATLAB:uistring:popupdialogs:Yes');
                cancel = vision.getMessage('MATLAB:uistring:popupdialogs:Cancel');
                buttonName = uiconfirm(this.AppContainer,question,title,...
                    'Options',{yes, cancel},...
                    'DefaultOption',1,'CancelOption',2);
                if ~strcmp(buttonName, yes)
                    return;
                end
            end
            
            % CTRL-DEL will also end up here
            idxMultiselect = selectedItem(this.BoardThumbnail);
            this.processRemoveAndRecalibrate(idxMultiselect);
        end
        
        % File selection handler
        %----------------------------------------
        function doSelection(this, ~, ~)
            
            if isImageCaptureTabInGroup(this) || isempty(this.Session.PatternSet)...
                    || this.Session.PatternSet.NumPatterns == 0
                return
            end
            
            if selectedItem(this.BoardThumbnail) < 1
                return;
            end
            
            % Poor man's lock. Set a flag indicating that doSelection()
            % is in progress to prevent doDeleteKey() from modifying
            % the session.
            this.StillDrawing = true;
            updateRemoveContextMenu(this);
            if isobjvalid(this.ReprojectionErrorsDisplay)
                resetSlider(this.ReprojectionErrorsDisplay);
            end
            
            % Set status text to indicate partial/complete checkerboard tag
            % for the current selection. Ignore for multiple selections.
            selection = selectedItem(this.BoardThumbnail);
            if numel(selection) == 1
                if this.Session.PatternSet.PartialPatterns(selection)
                    statusMsg = vision.getMessage('vision:caltool:PartialPatternStatus');
                else
                    statusMsg = vision.getMessage('vision:caltool:CompletePatternStatus');
                end
                this.setStatusText(statusMsg);
            else
                this.setStatusText('');
            end
            
            this.updatePlots();
            this.drawBoard();
            this.setFocusOnBoards();
            this.StillDrawing = false;
        end
        
        %------------------------------------------------------------------
        function processRemoveAndRecalibrate(this, idxMultiselect)
            
            this.Session.PatternSet.removePattern(idxMultiselect);
            this.Session.IsChanged = true;
            
            updateButtonStates(this);
            
            if this.Session.PatternSet.NumPatterns ~= 0
                this.BoardThumbnail.removeSelected(this.isStereoAppMode());

                newSelection = max(min(min(idxMultiselect)-1, this.BoardThumbnail.getNumImages()),1);
                this.BoardThumbnail.selectItem(newSelection);
            else
                this.resetAll;
            end
            
            % Update the UI before proceeding further
            drawnow;
            
            % Recalibrate
            if this.Session.isCalibrated()
                if this.Session.HasEnoughBoards
                    isOK = this.calibrate();
                    if ~isOK
                        this.resetCalibrationResults();
                    end
                else
                    this.resetCalibrationResults();
                end
            end
        end
        
        %------------------------------------------------------------------
        % Updates the current maximum reprojectionError in slider
        %------------------------------------------------------------------
        function val = getMaximumReprojectionError(this)
            if ~isempty(this.Session.CameraParameters)
                if ~this.IsStereo
                    val = max(mean(hypot(this.Session.CameraParameters.ReprojectionErrors(:,1,:),...
                        this.Session.CameraParameters.ReprojectionErrors(:,2,:))));
                else
                    val1 = max(mean(hypot(this.Session.CameraParameters.CameraParameters1.ReprojectionErrors(:,1,:),...
                        this.Session.CameraParameters.CameraParameters1.ReprojectionErrors(:,2,:))));
                    val2 = max(mean(hypot(this.Session.CameraParameters.CameraParameters2.ReprojectionErrors(:,1,:),...
                        this.Session.CameraParameters.CameraParameters2.ReprojectionErrors(:,2,:))));
                    val = max(val1,val2);
                end
            end
            
        end
        
        %------------------------------------------------------------------
        %  Gets the UI to the starting point, as if nothing has been loaded
        %------------------------------------------------------------------
        function resetAll(this)
            % reset the session
            this.Session.reset();
            
            % reset the message in the data browser
            this.clearBoards();
            this.ThumbnailDisplay.displayInitialDataBrowserMessage(this.IsStereo);
            
            % Reset the status text
            this.setStatusText('');
            
            % Not calibrated any more. Discard the tiled section
            if isobjvalid(this.ReprojectionErrorsDisplay)
                this.ReprojectionErrorsDisplay.close();
            end
            if isobjvalid(this.ExtrinsicsDisplayCamera)
                this.ExtrinsicsDisplayCamera.close();
            end
            if isobjvalid(this.ExtrinsicsDisplayPattern)
                this.ExtrinsicsDisplayPattern.close();
            end
            % wipe the visible figures
            this.MainImageDisplay.wipeFigure();
            
            % Reset the image capture tab.
            if ~isempty(this.ImageCaptureTab)
                this.ImageCaptureTab.resetAll();
            end
            
            % Reset the image properties dialog
            if ~isempty(this.ImagePropertiesDialog)
                this.ImagePropertiesDialog.close();
            end
            
            % update buttons
            updateButtonStates(this);
        end
        
        %------------------------------------------------------------------
        % Unlike resetAll(), this method resets all but the image data.
        % It will wipe the calibration results.
        %------------------------------------------------------------------
        function resetCalibrationResults(this)
            
            % wipe the calibration portion of the Session
            this.Session.resetCalibration();
            
            % Not calibrated any more. Discard the tiled section
            if isobjvalid(this.ReprojectionErrorsDisplay)
                this.ReprojectionErrorsDisplay.close();
            end
            if isobjvalid(this.ExtrinsicsDisplayCamera)
                this.ExtrinsicsDisplayCamera.close();
            end
            if isobjvalid(this.ExtrinsicsDisplayPattern)
                this.ExtrinsicsDisplayPattern.close();
            end
            
            updateButtonStates(this);
            
            % redraw the board since the reprojection data is no longer
            % available
            drawBoard(this);
        end
        
        %------------------------------------------------------------------
        % Calibration requires at least two boards. This routine grays out
        % the calibration button if there are fewer than MinBoards boards.
        %------------------------------------------------------------------
        function updateButtonStates(this)
            
            % Calibration tab
            if ~isempty(this.Session.PatternSet)
                if (this.Session.PatternSet.NumPatterns < this.MinBoards)
                    % gray out the calibration button
                    this.Session.HasEnoughBoards = false;
                else
                    this.Session.HasEnoughBoards = true;
                end
            end
            
            this.CalibrationTab.updateButtonStates(this.Session);
            
            if ~isempty(this.Session.PatternSet) && (this.Session.PatternSet.NumPatterns > 0)
                this.enableZoomButtons(true);
            else
                this.enableZoomButtons(false);
            end
        end
        
        %------------------------------------------------------------------
        function enableZoomButtons(this, enable)
            if enable
                if ~this.CalibrationTab.ZoomSection.IsEnabled
                    this.CalibrationTab.ZoomSection.enableButtons();
                    this.addZoomListeners();
                end
            else
                this.removeZoomListeners();
                this.CalibrationTab.ZoomSection.resetButtons();
                this.CalibrationTab.ZoomSection.disableButtons();
            end
            
        end
        
         %------------------------------------------------------------------
        % Dialog to get image and calibration pattern properties
        %------------------------------------------------------------------
        function createImagePropertiesDialog(this)
            location = getToolCenter(this);
            if this.IsStereo
                if isempty(this.LastImageDir)
                    this.ImagePropertiesDialog = vision.internal.calibration.webTool.LoadStereoImagesDlg(...
                        this, location);
                else
                    this.ImagePropertiesDialog = vision.internal.calibration.webTool.LoadStereoImagesDlg(...
                        this, location, this.LastImageDir{1},this.LastImageDir{2});
                end
            else
                this.ImagePropertiesDialog = vision.internal.calibration.webTool.ImagePropertiesDlg(...
                    this, location);
            end
        end
        
        %--------------------------------------------------------------
        function [initSquareSize, initUnits] = getInitialSquareSize(this)
            if isempty(this.Session.PatternSet)
                initSquareSize = 25;
                initUnits = 'millimeters';
            else
                initSquareSize = this.Session.PatternSet.SquareSize;
                initUnits = this.Session.PatternSet.Units;
            end
        end
        
        %--------------------------------------------------------------
        function doOKKeyPress(~, ~, evd)
            
            switch(evd.Key)
                case {'return','space','escape'}
                    uiresume(gcbf);
            end
        end
        
        %------------------------------------------------------------------
        function updatePlots(this)
            % Unlike drawPlots, updatePlots only update necessary part of plots
            % without redrawing it for performance improvement.
            if this.Session.isCalibrated() % has calibration results
                updateSelection(this.ReprojectionErrorsDisplay,this.getHighlightIndex());
                updateSelection(this.ExtrinsicsDisplayCamera,this.getHighlightIndex());
                updateSelection(this.ExtrinsicsDisplayPattern,this.getHighlightIndex());
                
                drawnow;
            else
                if isobjvalid(this.ReprojectionErrorsDisplay)
                    this.ReprojectionErrorsDisplay.lockFigure();
                end
                if isobjvalid(this.ExtrinsicsDisplayCamera)
                    this.ExtrinsicsDisplayCamera.lockFigure();
                end
                if isobjvalid(this.ExtrinsicsDisplayPattern)
                    this.ExtrinsicsDisplayPattern.lockFigure();
                end
            end
        end
        
        %------------------------------------------------------------------
        function drawPlots(this)
            
            if this.Session.isCalibrated() % has calibration results
                this.plotExtrinsics;
                this.plotErrors;
            else
                this.ReprojectionErrorsDisplay.lockFigure();
                this.ExtrinsicsDisplayCamera.lockFigure();
                this.ExtrinsicsDisplayPattern.lockFigure();
            end
        end
        
        %------------------------------------------------------------------
        function highlightIndex = getHighlightIndex(this)
            
            if this.Session.isCalibrated()
                % This can happen when adding new images to the bottom
                % of the image stack.  In that case, we do not have
                % anything to highlight for the brand new boards
                boardIndex = selectedItem(this.BoardThumbnail);
                highlightIndex = ...
                    boardIndex(boardIndex <= ...
                    this.Session.CameraParameters.NumPatterns);
            else
                highlightIndex = [];
            end
        end
        
        %------------------------------------------------------------------
        function plotExtrinsics(this, varargin)
            displayFigureCamera = this.ExtrinsicsDisplayCamera;
            displayFigurePattern = this.ExtrinsicsDisplayPattern;
            
            isFigureClosed = ~ishandle(displayFigureCamera.Figure) && ...
                ~ishandle(displayFigurePattern.Figure);
            if isFigureClosed
                return;
            end
            
            if ~isAxesValid(displayFigurePattern)
                displayFigurePattern.createAxes();
            end
            
            if ~isAxesValid(displayFigureCamera)
                displayFigureCamera.createAxes();
            end
            
            displayFigurePattern.switchView('PatternCentric');
            plotGraph(this, displayFigurePattern);
            displayFigureCamera.switchView('CameraCentric');
            plotGraph(this, displayFigureCamera);
            drawnow();
        end
        
        %------------------------------------------------------------------
        function plotErrors(this, varargin)
            displayFigure = this.ReprojectionErrorsDisplay;
            % What if the figure has been closed?
            if ~ishandle(displayFigure.Figure)
                return;
            end
            
            if ~isAxesValid(displayFigure)
                displayFigure.createAxes();
            end
            
            plotGraph(this, displayFigure);
            set(displayFigure.Figure, 'KeyPressFcn',@(evt,data)this.doDeleteKey(evt,data));
            drawnow();
        end
        
        %--------------------------------------------------------------
        function plotGraph(this, displayFigure)
            plot(displayFigure, ...
                this.Session.CameraParameters, this.getHighlightIndex(), ...
                @(h, ~)onClickPlot(this, displayFigure, h), ...
                @(h, ~)onClickPlotSelected(this, displayFigure, h));
        end
        %{
        %--------------------------------------------------------------
        function onSwitchView(this, displayFigure, newView)
            displayFigure.switchView(newView);
            plotGraph(this, displayFigure);
        end
        %}
        %------------------------------------------------------------------
        function onClickPlot(this, displayFigure, h)
            [clickedIdx, selectionType] = getSelection(displayFigure, h);
            processClick(this, selectionType, clickedIdx);
        end
        
        %------------------------------------------------------------------
        function onClickPlotSelected(this, displayFigure, h)
            [clickedIdx, selectionType] = getSelection(displayFigure, h);
            processSelectedClick(this, selectionType, clickedIdx);
        end
        
        %------------------------------------------------------------------
        function processClick(this,selectionType,clickedIdx)
            % Reset the threshold line location
            % do this first because the cost is cheap
            resetSlider(this.ReprojectionErrorsDisplay);
         
            switch(selectionType)
                case 'alt'
                    % control-click or right-click
                    this.BoardThumbnail.setSelectedIndices(clickedIdx);
                case 'normal'
                    % plain click
                    this.BoardThumbnail.selectItem (clickedIdx);
                case 'extend'
                    % shift-click
                    prevIdx = selectedItem(this.BoardThumbnail);
                    dists = abs(prevIdx - clickedIdx);
                    [~, prevIdxIdx] = min(dists);
                    prevIdx = prevIdx(prevIdxIdx);
                    nextIdx = 1;
                    if prevIdx > clickedIdx
                        nextIdx = -1;
                    end
                    
                    this.BoardThumbnail.setSelectedIndices(...
                        (min(prevIdx+nextIdx,clickedIdx):max(prevIdx+nextIdx,clickedIdx)));
            end
            
            this.updatePlots();
            this.drawBoard();
            this.setFocusOnBoards();
            
            drawnow;
        end
        
        %------------------------------------------------------------------
        function processSelectedClick(this,selectionType,clickedIdx)
            
            switch(selectionType)
                case 'alt'
                    % control-click or right-click on a selected bar should
                    % deselect it
                    selectedIdx = selectedItem(this.BoardThumbnail);
                    if numel(selectedIdx) > 1
                        selectedIdx(selectedIdx == clickedIdx) = [];
                        this.BoardThumbnail.selectItem (selectedIdx);
                    end
                case 'normal'
                    this.BoardThumbnail.selectItem (clickedIdx);
                case 'extend'
                    prevIdx = selectedItem(this.BoardThumbnail);
                    dists = abs(prevIdx - clickedIdx);
                    dists(dists == 0) = inf;
                    [~, prevIdxIdx] = min(dists);
                    prevIdx = prevIdx(prevIdxIdx);
                    nextIdx = 1;
                    if prevIdx > clickedIdx
                        nextIdx = -1;
                    end
                    this.BoardThumbnail.setSelectedIndices(...
                        (min(prevIdx+nextIdx,clickedIdx):max(prevIdx+nextIdx,clickedIdx)));
            end
        end
        
        %------------------------------------------------------------------
        function outSession = getSession(this)
            outSession = this.Session;
        end
        
        %------------------------------------------------------------------
        function addToolInstance(this)
            this.manageToolInstances('add', this);
        end
        
        %------------------------------------------------------------------
        function doActionCallback(this, group, ~)
            import matlab.ui.container.internal.appcontainer.*;
            
            switch group.State
                case {AppState.RUNNING, AppState.INITIALIZING}
                    doAppFocusRestored(this);

                case AppState.TERMINATED
                    % Add components that need to close while app is
                    % closing

                    % The main object is already getting deleted so we
                    % can't continue with deleting the rest of its
                    % sub-components.
                    if ~isvalid(this)
                        return;
                    end

                    if isobjvalid(this.ImagePropertiesDialog)
                        delete(this.ImagePropertiesDialog.FigureHandle);
                    end

                    if isobjvalid(this.ImageStatsDlg)
                        if isobjvalid(this.ImageStatsDlg.RejectedImagesDlg)
                            delete(this.ImageStatsDlg.RejectedImagesDlg.FigureHandle);
                        end
                        delete(this.ImageStatsDlg.FigureHandle);
                    end

                    if isobjvalid(this.ExportDlg)
                        delete(this.ExportDlg.FigureHandle);
                    end

                    if isobjvalid(this.LoadDlg)
                        delete(this.LoadDlg.FigureHandle);
                    end

                    if isobjvalid(this.OptimizationOptionDlg)
                        delete(this.OptimizationOptionDlg.FigureHandle);
                    end
                    
            end
        end
        
        %------------------------------------------------------------------
        function doAppFocusRestored(this)
            if isvalid(this.AppContainer)
                notify(this, 'AppFocusRestored');
            end
        end
        
        %------------------------------------------------------------------
        function doAppFocusLost(this)
            if isvalid(this.AppContainer)
                notify(this, 'AppFocusLost');
            end
        end
        
        %------------------------------------------------------------------
        function doAddImagesToNewSession(this, evtData)
            detector = evtData.PatternDetector;
            detectorFile = evtData.PatternDetectorFile;
            
            if this.IsStereo
                files = evtData.FileNames;
                this.Files = files;
                this.LastImageDir{1} = evtData.Dir1;
                this.LastImageDir{2} = evtData.Dir2;
                isUserCanceled = isempty(files);
                
                if ~isUserCanceled
                   addImagesToNewSession(this, files, detector, detectorFile);
                end
            else
                files = this.Files;
                addImagesToNewSession(this, files, detector, detectorFile);
                
                if this.ImageCaptureFlag
                    this.ImageCaptureFlag = false;
                    notify(this.ImageCaptureTab, 'UpdateTab');
                end
            end
        end
        
        %------------------------------------------------------------------
        function doAddImagesToExistingSession(this, evtData)
            files = evtData.FileNames;
            this.Files = [this.Files files];
            this.LastImageDir{1} = evtData.Dir1;
            this.LastImageDir{2} = evtData.Dir2;
            isUserCanceled = isempty(files);
            
            if ~isUserCanceled
                addImagesToExistingSession(this, files);
            end
        end
        
        %------------------------------------------------------------------
        function doImagePropertiesClosed(this)
            this.clearBoards();
            this.ThumbnailDisplay.displayInitialDataBrowserMessage(this.IsStereo);
            
            if this.ImageCaptureFlag
                this.ImageCaptureFlag = false;
                notify(this.ImageCaptureTab, 'UpdateTab');
            end
        end
        %------------------------------------------------------------------
        function closeApproval = closingSession(this, toolManageFlag)
            
            if isImageCaptureTabInGroup(this)
                closePreview(this.ImageCaptureTab);
            end
            
            sessionChanged = this.Session.IsChanged;
            
            yes    = vision.getMessage('MATLAB:uistring:popupdialogs:Yes');
            no     = vision.getMessage('MATLAB:uistring:popupdialogs:No');
            cancel = vision.getMessage('MATLAB:uistring:popupdialogs:Cancel');
            
            if sessionChanged
                selection = this.askForSavingOfSession(this.AppContainer);
            else
                selection = no;
            end
            
            switch selection
                case yes
                    success = this.saveSession();
                    if success
                        closeApproval = true;
                        this.deleteToolInstance(toolManageFlag);
                    else
                        closeApproval = false;
                    end
                case no
                    closeApproval = true;
                    this.deleteToolInstance(toolManageFlag);
                case cancel
                    closeApproval = false;
                otherwise
                    closeApproval = false;
            end
        end
        
        %------------------------------------------------------------------
        function createDefaultLayout(this)
            
            % create all the required figures
            if isempty(this.ThumbnailDisplay)
                this.ThumbnailDataBrowser = matlab.ui.internal.FigurePanel;
                this.ThumbnailDataBrowser.Figure.Color = matlab.graphics. ...
                    internal.themes.getAttributeValue(this.Theme,...
                    '--mw-backgroundColor-input');
                this.ThumbnailDisplay = ...
                    vision.internal.calibration.webTool.ThumbnailDisplay(this.ThumbnailDataBrowser);
                this.AppContainer.add(this.ThumbnailDataBrowser);
            else
                this.ThumbnailDisplay.makeFigureVisible();
            end
            
            configureImageDisplay(this);
            
            % If image capture tab is in the toolgroup, bring
            % focus to it.
            if isImageCaptureTabInGroup(this)
                this.ImagePreviewDisplay.Showing = true;
            end

            if this.Session.isCalibrated()
                createTiledSection(this);
            else
                fullFileName = fullfile(toolboxdir('vision'),'vision',...
                    '+vision','+internal','+calibration','+webTool','InitialLayout.mat');

                layout = load(fullFileName);
                this.AppContainer.Layout = layout.layout;
            end
        end
        
        %------------------------------------------------------------------
        function configurePreviewDisplay(this)
            
            tabName = vision.getMessage('vision:uitools:MainPreviewFigure');
            if isempty(this.CalibrationPreviewDocGroup)
                figDocGroup.DefaultRegion = "right";
                figDocGroup.Title = "Calibration Preview Display";
                figDocGroup.Tile = 1;
                figDocGroup.Tag = "CalibrationPreviewDisplayGroup";
                
                % Create contaxt Defintion for document group
                figDocGroup.Context = matlab.ui.container.internal.appcontainer.ContextDefinition();
                
                % Attach Crop Image Tab Toolstrip to document group
                figDocGroup.Context.ToolstripTabGroupTags = "ImageCaptureTabGroup";
                
                % Create Document group and add it to container
                this.CalibrationPreviewDocGroup = matlab.ui.internal.FigureDocumentGroup(figDocGroup);
                this.AppContainer.add(this.CalibrationPreviewDocGroup);
            end
            
            % Create figure document and attach it to Document group.
            if this.IsStereo
                mainImageTag = "Stereo Camera Calibrator_Camera";
            else
                mainImageTag = "Camera Calibrator_Camera";
            end
            imageDisplay.Tag = mainImageTag;
            imageDisplay.Title = tabName;
            imageDisplay.DocumentGroupTag = "CalibrationPreviewDisplayGroup";
            
            this.ImagePreviewDisplay = ...
                vision.internal.calibration.webTool.ImagePreview(imageDisplay);
            
            % Remove 'x' from document Figure. This will protect closing of
            % figure from user
            this.ImagePreviewDisplay.Closable = false;
            this.ImagePreviewDisplay.Tile = 1;
            this.AppContainer.add(this.ImagePreviewDisplay);
        end
        
        %------------------------------------------------------------------
        function configureImageDisplay(this)

            if isempty(this.MainImageDisplay)
                tabName = vision.getMessage('vision:uitools:MainImageFigure');
                if isempty(this.CalibrationImageDocGroup)
                    figDocGroup.DefaultRegion = "right";
                    figDocGroup.Title = "Calibration Image Display";
                    figDocGroup.Tile = 1;
                    figDocGroup.Tag = "CalibrationImageDisplayGroup";

                    % Create Document group and add it to container
                    this.CalibrationImageDocGroup = matlab.ui.internal.FigureDocumentGroup(figDocGroup);
                    this.AppContainer.add(this.CalibrationImageDocGroup);
                end

                % Create figure document and attach it to Document group.
                if this.IsStereo
                    mainImageTag = "Stereo Camera Calibrator_Image";
                else
                    mainImageTag = "Camera Calibrator_Image";
                end
                imageDisplay.Tag = mainImageTag;
                imageDisplay.Title = tabName;
                imageDisplay.DocumentGroupTag = "CalibrationImageDisplayGroup";

                if this.IsStereo
                    this.MainImageDisplay = ...
                        vision.internal.calibration.webTool.StereoCalibrationImageDisplay(imageDisplay);
                else
                    this.MainImageDisplay = ...
                        vision.internal.calibration.webTool.SingleCalibrationImageDisplay(imageDisplay);
                end

                % Remove 'x' from document Figure. This will protect closing of
                % figure from user
                this.MainImageDisplay.Closable = false;
                this.AppContainer.add(this.MainImageDisplay);
            end
        end
        
        %------------------------------------------------------------------
        function tf = isImageCaptureTabInGroup(this)
            
            if isempty(this.ImageCaptureTab)
                
                tf = false;
            else
                if ~isempty(this.ImageCaptureTab.TabGroup.SelectedTab) && ...
                        isempty(this.CalibrationTab.TabGroup.SelectedTab)
                    tf = true;
                else
                    tf = false;
                end
            end
        end
        
        %------------------------------------------------------------------
        function createTiledSection(this)
            % Create the tiled section for reprojection and extrinsics plot

            if isempty(this.ThumbnailDisplay)
                this.ThumbnailDataBrowser = matlab.ui.internal.FigurePanel;
                this.ThumbnailDisplay = ...
                    vision.internal.calibration.webTool.ThumbnailDisplay(this.ThumbnailDataBrowser);
                this.AppContainer.add(this.ThumbnailDataBrowser);
            end

            if isempty(this.CalibrationPatternDisplayDocGroup)
                figDocGroup.DefaultRegion = "right";
                figDocGroup.Title = vision.getMessage('vision:caltool:CalibrationPatternDisplay');
                figDocGroup.Tile = 1;
                figDocGroup.Tag = "CalibrationPatternDisplayGroup";

                % Create Document group and add it to container
                this.CalibrationPatternDisplayDocGroup = matlab.ui.internal.FigureDocumentGroup(figDocGroup);
                this.AppContainer.add(this.CalibrationPatternDisplayDocGroup);
            end

            if isempty(this.ReprojectionErrorsDisplay) || ~isvalid(this.ReprojectionErrorsDisplay)...
                    || ~this.ReprojectionErrorsDisplay.isAxesValid

                % Create figure document and attach it to Document group.
                imageDisplay.Tag = "ReprojectionErrorsAxes";
                imageDisplay.Title = getString(message('vision:caltool:ErrorsFigure'));
                imageDisplay.DocumentGroupTag = this.CalibrationPatternDisplayDocGroup.Tag;

                this.ReprojectionErrorsDisplay = ...
                    vision.internal.calibration.webTool.ReprojectionErrorsDisplay(imageDisplay);
                this.AppContainer.add(this.ReprojectionErrorsDisplay);

                addlistener(this.ReprojectionErrorsDisplay,'ErrorPlotChanged',@(~,~)this.updateSelection);
            end


            if isempty(this.ExtrinsicsDisplayPattern) || ~isvalid(this.ExtrinsicsDisplayPattern)...
                    || ~this.ExtrinsicsDisplayPattern.isAxesValid
                title = vision.getMessage('vision:caltool:ExtrinsicsFigurePattern');

                % Create figure document and attach it to Document group.
                imageDisplay.Tag = "ExtrinsicsAxesPattern";
                imageDisplay.Title = title;
                imageDisplay.DocumentGroupTag = this.CalibrationPatternDisplayDocGroup.Tag;

                this.ExtrinsicsDisplayPattern = ...
                    vision.internal.calibration.webTool.ExtrinsicsDisplay(imageDisplay);
                this.AppContainer.add(this.ExtrinsicsDisplayPattern);
            end

            if isempty(this.ExtrinsicsDisplayCamera) || ~isvalid(this.ExtrinsicsDisplayCamera)...
                    || ~this.ExtrinsicsDisplayCamera.isAxesValid
                title = vision.getMessage('vision:caltool:ExtrinsicsFigureCamera');

                % Create figure document and attach it to Document group.
                imageDisplay.Tag = "ExtrinsicsAxesCamera";
                imageDisplay.Title = title;
                imageDisplay.DocumentGroupTag = this.CalibrationPatternDisplayDocGroup.Tag;

                this.ExtrinsicsDisplayCamera = ...
                    vision.internal.calibration.webTool.ExtrinsicsDisplay(imageDisplay);
                this.AppContainer.add(this.ExtrinsicsDisplayCamera);
            end

            % Load Layout file
            if this.IsStereo

                fullFileName = fullfile(toolboxdir('vision'),'vision',...
                    '+vision','+internal','+calibration','+webTool','StereoCameraCalibratorLayout.mat');
            else
                if isImageCaptureTabInGroup(this)

                    fullFileName = fullfile(toolboxdir('vision'),'vision',...
                        '+vision','+internal','+calibration','+webTool','CameraCaptureLayout.mat');
                else
                    
                    fullFileName = fullfile(toolboxdir('vision'),'vision',...
                        '+vision','+internal','+calibration','+webTool','CameraCalibratorLayout.mat');
                end
            end

            appLayout = load(fullFileName);
            this.AppContainer.Layout = appLayout.layout;

            this.MainImageDisplay.makeFigureVisible();
            this.ReprojectionErrorsDisplay.makeFigureVisible();

            % Order important to make Camera-Centric view the one 'on top'.
            this.ExtrinsicsDisplayPattern.makeFigureVisible();
            this.ExtrinsicsDisplayCamera.makeFigureVisible();

            if ~this.IsStereo && isImageCaptureTabInGroup(this)
                % Set focus to imageCaptureTab
                this.ImagePreviewDisplay.Showing = true;
            end
        end

        %------------------------------------------------------------------
        function createMonoDocumentLayoutStruct(this)

            % Create Empty Structure
            documentLayout = struct;

            % Add Grid Dimensions
            documentLayout.gridDimensions.w = 2;
            documentLayout.gridDimensions.h = 2;

            % A total of 3 tiles
            documentLayout.tileCount = 3;

            % Set colum and row weights
            documentLayout.columnWeights = [0.72 0.28];

            % Set tile coverage Tile 1 and Tile 2 occupy row 1 column 1,
            % and row 2 column 2 respectively. And Tile 3 occupy all of the
            % second row.
            documentLayout.tileCoverage = [1 2; 1 3];

            % Define which documents appear in which tile Id of a document
            % is defined as <GroupTag>_<DocumentTag>
            document1State.id = this.CalibrationImageDocGroup.Tag ...
                + "_" + this.MainImageDisplay.Tag;
            document2State.id = this.CalibrationPatternDisplayDocGroup.Tag ...
                + "_" + this.ReprojectionErrorsDisplay.Tag;
            document3State.id = this.CalibrationPatternDisplayDocGroup.Tag ...
                + "_" + this.ExtrinsicsDisplayPattern.Tag;
            document4State.id = this.CalibrationPatternDisplayDocGroup.Tag ...
                + "_" + this.ExtrinsicsDisplayCamera.Tag;

            tile1Children = [document1State];
            tile2Children = [document2State];
            tile3Children = [document3State, document4State];

            tile1Occupancy.children = tile1Children;
            tile2Occupancy.children = tile2Children;
            tile3Occupancy.children = tile3Children;

            documentLayout.tileOccupancy = [tile1Occupancy tile2Occupancy tile3Occupancy];

            % Set document layout on app
            this.AppContainer.DocumentLayout = documentLayout;
        end

        %------------------------------------------------------------------
        function createStereoDocumentLayoutStruct(this)

            % Create Empty Structure
            documentLayout = struct;

            % Add Grid Dimensions
            documentLayout.gridDimensions.w = 2;
            documentLayout.gridDimensions.h = 2;

            % A total of 3 tiles
            documentLayout.tileCount = 3;

            % Set colum and row weights
            documentLayout.rowWeights = [0.6, 0.4];

            % Set tile coverage Tile 1 and Tile 2 occupy row 1 column 1,
            % and row 2 column 2 respectively. And Tile 3 occupy all of the
            % second row.
            documentLayout.tileCoverage = [1, 1; 2, 3];

            % Define which documents appear in which tile Id of a document
            % is defined as <GroupTag>_<DocumentTag>
            document1State.id = this.CalibrationImageDocGroup.Tag ...
                + "_" + this.MainImageDisplay.Tag;
            document2State.id = this.CalibrationPatternDisplayDocGroup.Tag ...
                + "_" + this.ReprojectionErrorsDisplay.Tag;
            document3State.id = this.CalibrationPatternDisplayDocGroup.Tag ...
                + "_" + this.ExtrinsicsDisplayPattern.Tag;
            document4State.id = this.CalibrationPatternDisplayDocGroup.Tag ...
                + "_" + this.ExtrinsicsDisplayCamera.Tag;

            tile1Children = [document1State];
            tile2Children = [document2State];
            tile3Children = [document3State, document4State];

            tile1Occupancy.children = tile1Children;
            tile2Occupancy.children = tile2Children;
            tile3Occupancy.children = tile3Children;

            documentLayout.tileOccupancy = [tile1Occupancy tile2Occupancy tile3Occupancy];

            % Set document layout on app
            this.AppContainer.DocumentLayout = documentLayout;
        end
        
        %------------------------------------------------------------------
        function updateSelection(this)
            % Update selected images based on reprojection error display
            this.BoardThumbnail.removeSelectionListener();
            indx = this.ReprojectionErrorsDisplay.getSelected();
            this.BoardThumbnail.selectItem (indx);
            this.BoardThumbnail.addSelectionListener();
            
            this.updatePlots()
            this.drawBoard();
        end
        
        %------------------------------------------------------------------
        function loc = getToolCenter(this)
            loc = imageslib.internal.app.utilities.ScreenUtilities.getToolCenter(this.AppContainer);
        end

        %------------------------------------------------------------------
        function setTheme(this)
            if useDarkTheme
                this.Theme = matlab.graphics.internal.themes.darkTheme;
            else
                this.Theme = matlab.graphics.internal.themes.lightTheme;
            end
        end
    end %Smaller Toolstrip Button Callbacks
    
    %------------------------------------------------------------------
    methods(Access = protected)
        %------------------------------------------------------------------
        % Delete method for App-container based apps. This implementation
        % handles the work required for most apps. Namely, deleting the
        % main Appcontainer and all the figures associated with the app.
        % -----------------------------------------------------------------
        function closeApp(this)
            
            this.closeAllFigures(false); % shut down all figures
            this.AppContainer.delete; % close the UI
            
            drawnow();  % allow time for closing of all figures
        end
        
        %------------------------------------------------------------------
        function closeAllFigures(this, closeFigureDocument)
            % closeAllFigures performs closeing of App and layout update. 
            % Use 'closeFigureDocument = true' for layout update which
            % hides thumbnails and close all figure documents and recreate
            % it.
            % Use 'closeFigureDocument = false' to delete and remove
            % thumbnail and avoid the closing of figure documents which
            % will be closed with the AppContainer.
            
            if closeFigureDocument
                this.ThumbnailDisplay.makeFigureInvisible();
            else
                if ~isempty(this.ThumbnailDisplay)
                    this.ThumbnailDisplay.close();
                end
                if ~isempty(this.BoardThumbnail)
                    deleteBrowser(this.BoardThumbnail);
                end

                if ~isempty(this.ImageCaptureTab)
                    deleteCameraObject(this.ImageCaptureTab)
                end

                if isobjvalid(this.ImagePropertiesDialog)
                   delete(this.ImagePropertiesDialog.FigureHandle); 
                end
            end
            
            if closeFigureDocument
                % clean up the preview figure
                if isobjvalid(this.ImagePreviewDisplay)

                    % If image capture tab is not in the toolgroup, add it and
                    % bring focus to it.
                    if isImageCaptureTabInGroup(this)
                        this.ImageCaptureTab.closePreview();
                    end
                    this.ImagePreviewDisplay.close();
                end

                % clean up the figures
                this.MainImageDisplay.close();

                if isobjvalid(this.ReprojectionErrorsDisplay)
                    this.ReprojectionErrorsDisplay.close();
                end
                if isobjvalid(this.ExtrinsicsDisplayCamera)
                    this.ExtrinsicsDisplayCamera.close();
                end
                if isobjvalid(this.ExtrinsicsDisplayPattern)
                    this.ExtrinsicsDisplayPattern.close();
                end
            end
        end
    end
    %----------------------------------------------------------------------
    % Static public methods
    %----------------------------------------------------------------------
    methods (Static)
        
        %------------------------------------------------------------------
        function deleteAllTools
            vision.internal.calibration.webTool.CameraCalibrationTool.manageToolInstances('deleteAll');
        end
        
        %------------------------------------------------------------------
        function deleteAllToolsForce
            vision.internal.calibration.webTool.CameraCalibrationTool.manageToolInstances('deleteAllForce');
        end
    end
    
    %----------------------------------------------------------------------
    % Static private methods
    %----------------------------------------------------------------------
    methods (Access='private', Static)
        %------------------------------------------------------------------
        % Manages a persistent variable for the purpose of tracking the
        % tool instances.
        %------------------------------------------------------------------
        function manageToolInstances(action, varargin)
            
            mlock();
            
            persistent toolArray;
            
            switch action
                case 'add'
                    if isempty(toolArray) % first time
                        toolArray = varargin{1};
                    else
                        % add to existing array
                        toolArray(end+1) = varargin{1};
                    end
                case 'delete'
                    for i=1:length(toolArray)
                        this = varargin{1};
                        if strcmp(this.AppContainer.Tag, toolArray(i).AppContainer.Tag)
                            if ~(nargin == 3 && varargin{2} == false)
                                closeAllFigures(toolArray(i), false);
                                toolArray(i) = [];
                            end
                            break;
                        end
                    end
                    
                case 'deleteAll'
                    % wipe backwards since toolArray will be shrinking
                    for i = length(toolArray):-1:1
                        closingSession(toolArray(i), false);
                        closeApp(toolArray(i));
                    end
                    toolArray = [];
                    
                case 'deleteAllForce'
                    % wipe backwards since toolArray will be shrinking
                    for i = length(toolArray):-1:1
                        toolArray(i).AppContainer.CanCloseFcn = [];
                        closeApp(toolArray(i));
                    end
                    toolArray = [];
            end
            
            % If all tools are closed, permit clearing of the class. This
            % is helpful during development of the tool
            if isempty(toolArray)
                munlock();
            end
            
        end
        
    end
    
    %----------------------------------------------------------------------
    methods(Static, Access = protected)
        %------------------------------------------------------------------
        % Pops up dialog asking if session should be saved. Returns the
        % dialog selection: yes, no, or cancel. Should be called during
        % when closing a session or creating a new session when a session
        % already open.
        %------------------------------------------------------------------
        function selection = askForSavingOfSession(appContainer)
            
            yes    = vision.getMessage('MATLAB:uistring:popupdialogs:Yes');
            no     = vision.getMessage('MATLAB:uistring:popupdialogs:No');
            cancel = vision.getMessage('MATLAB:uistring:popupdialogs:Cancel');
            
            selection = uiconfirm(appContainer,...
                vision.getMessage('vision:uitools:SaveSessionQuestion'),...
                vision.getMessage('vision:uitools:SaveSessionTitle'),...
                'Options',{yes, no, cancel},...
                'DefaultOption',2,'CancelOption',3);
            
            if isempty(selection) % dialog was destroyed with a click
                selection = cancel;
            end
        end
    end
    
end

%--------------------------------------------------------------------------
function flag = isobjvalid(obj)
    % Check whether object is empty or deleted
    flag = ~isempty(obj) && isvalid(obj);
end

%--------------------------------------------------------------------------
function tf = useDarkTheme()
    s = settings;
    mode = s.matlab.appearance.MATLABTheme.ActiveValue;
    tf = strcmpi(mode,'Dark');
end
