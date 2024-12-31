% CalibrationTab Defines key UI elements of the Camera Calibrator App
%
%    This class defines all key UI elements and sets up callbacks that
%    point to methods inside the CameraCalibrationTool class.

% Copyright 2012-2023 The MathWorks, Inc.

classdef CalibrationTab < handle
    
    properties
        FileSection;
        ZoomSection;
        LayoutSection;
        IntrinsicsOptionsSection;
        CameraOptionsSection;
        OptimizationOptionsSection;
        CalibrateSection;
        ViewUndistortSection;
        ExportSection;
        StandardOptionsSection;
        
        % Popup and panels for cameraCalibrator.
        StandardOptionsPanel;
        FisheyeOptionsPanel;
        OptionsPopup;
    end
    
    %----------------------------------------------------------------------
    % App Container Tab
    %----------------------------------------------------------------------
    properties
        Tab
        TabGroup
        Parent
    end
    
    properties(Dependent)
        CameraModel;
        CurrentCameraModelPanel;
    end
    
    %----------------------------------------------------------------------
    % Public methods
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        % Gets the camera model options
        %------------------------------------------------------------------
        function cameraModel = get.CameraModel(this)
            cameraModel = this.CurrentCameraModelPanel.CameraModel;
        end
        
        %------------------------------------------------------------------
        function currentCameraPanel = get.CurrentCameraModelPanel(this)
            parent = this.Parent;
            isStereo = parent.isStereoAppMode();
            if ~isStereo
                if this.CameraOptionsSection.IsFisheyeSelected
                    currentCameraPanel = this.FisheyeOptionsPanel;
                else
                    currentCameraPanel = this.StandardOptionsPanel;
                end
            else
                currentCameraPanel = this.StandardOptionsSection;
            end
        end
        
        %------------------------------------------------------------------
        function set.CurrentCameraModelPanel(this, currentCameraPanel)
            parent = this.Parent;
            isStereo = parent.isStereoAppMode();
            if ~isStereo
                if this.CameraOptionsSection.IsFisheyeSelected
                    this.FisheyeOptionsPanel.CameraModel = currentCameraPanel.CameraModel;
                else
                    this.StandardOptionsPanel.CameraModel = currentCameraPanel.CameraModel;
                end
            else
                this.StandardOptionsSection.CameraModel = currentCameraPanel.CameraModel;
            end
        end
    end
    
    methods (Access=public)
        %------------------------------------------------------------------
        % Constructor
        %------------------------------------------------------------------
        function this = CalibrationTab(tool, isStereo)
            % Create Tab group
            this.TabGroup = matlab.ui.internal.toolstrip.TabGroup();
            this.TabGroup.Tag = 'CalibrationTabGroup';
            
            tabName = getString(message('vision:caltool:CalibrationTab'));
            
            % Create and add Tab to Tabgroup
            this.Tab = matlab.ui.internal.toolstrip.Tab(tabName);
            this.Tab.Tag = strcat(tool.AppContainer.Title,'_',tabName);
            this.TabGroup.add(this.Tab);
            
            % Required for internal purpose
            this.Parent = tool;
            
            this.createWidgets(isStereo);
            this.installListeners(isStereo);
        end
        
        %------------------------------------------------------------------
        function testers = getTesters(~)
            testers = [];
        end
        
        %------------------------------------------------------------------
        % This routine handles graying out of buttons
        %------------------------------------------------------------------
        function updateButtonStates(this, session)
            parent = this.Parent;
            isStereo = parent.isStereoAppMode();
            if isStereo
                updateIntrinsicsButtonState(this, session);
            end
            if ~hasLoadedImages(parent)
                if isStereo
                    this.IntrinsicsOptionsSection.setAllButtonsEnabled(false);
                    this.StandardOptionsSection.setAllButtonsEnabled(false);
                    this.OptimizationOptionsSection.disableButton();
                else
                    this.CameraOptionsSection.setAllButtonsEnabled(false);
                end
            else
                if isStereo
                    if ~session.IsFixedIntrinsics
                        this.IntrinsicsOptionsSection.enableRadioButtons(true);
                        this.OptimizationOptionsSection.enableButton();
                    end
                else
                    this.CameraOptionsSection.setAllButtonsEnabled(true);
                end
            end
            updateCalibrateButtonState(this, session);
            updateViewUndistortButtonState(this, session);
            updateExportButtonState(this, session);
        end
        
        %------------------------------------------------------------------
        % This routine handles enabling/disabling everything in calibration tab.
        %------------------------------------------------------------------
        function updateTabStatus(this, state)
            updateExportButtonState(this.ExportSection, state);
            this.FileSection.setAllButtonsEnabled(state);
            this.CameraOptionsSection.setAllButtonsEnabled(state);
            this.CalibrateSection.IsButtonEnabled = state;
            this.LayoutSection.IsButtonEnabled = state;
        end
        
        %------------------------------------------------------------------
        % Sets the camera model options
        %------------------------------------------------------------------
        function setCameraModelOptions(this, isStereo, numRadialCoeffs, ...
                computeSkew, computeTangentialDist, isFisheyeSelected, ...
                estimateAlignment)
            
            % Standard camera model parameters
            standardCameraModel.NumDistortionCoefficients = numRadialCoeffs;
            standardCameraModel.ComputeSkew = computeSkew;
            standardCameraModel.ComputeTangentialDistortion = computeTangentialDist;
            
            % Fisheye camera model parameter
            fisheyeCameraModel.EstimateAlignment = estimateAlignment;
            
            if ~isStereo
                if isFisheyeSelected
                    this.CameraOptionsSection.selectFisheyeModel();
                else
                    this.CameraOptionsSection.selectStandardModel();
                end
                this.StandardOptionsPanel.CameraModel = standardCameraModel;
                this.FisheyeOptionsPanel.CameraModel = fisheyeCameraModel;
            else
                this.StandardOptionsSection.CameraModel = standardCameraModel;
            end
        end
        
        %------------------------------------------------------------------
        function setDefaultIntrinsicsOptions(this)
            % Default mode is to compute camera intrinsics.
            this.IntrinsicsOptionsSection.selectComputeMode();
        end
        
        %------------------------------------------------------------------
        function enableNumRadialCoefficients(this)
            parent = this.Parent;
            isStereo = parent.isStereoAppMode();
            if isStereo
                this.StandardOptionsSection.enableNumRadialCoefficients();
            else
                this.StandardOptionsPanel.enableNumRadialCoefficients();
            end
        end
        
        %------------------------------------------------------------------
        function disableNumRadialCoefficients(this, numCoeffs)
            parent = this.Parent;
            isStereo = parent.isStereoAppMode();
            if isStereo
                this.StandardOptionsSection.disableNumRadialCoefficients(numCoeffs);
            else
                this.StandardOptionsPanel.disableNumRadialCoefficients(numCoeffs);
            end
        end

        %------------------------------------------------------------------
        function installRadialCoeffsButtonListener(this)

            parent = this.Parent;
            isStereo = parent.isStereoAppMode();
            fun = @(es,ed)cameraModelChanged(this.Parent);
            if isStereo
                this.StandardOptionsSection.RadialCoeffsButton1.ValueChangedFcn = fun;
                this.StandardOptionsSection.RadialCoeffsButton2.ValueChangedFcn  = fun;
            else
                this.StandardOptionsPanel.installRadialCoeffsButtonListener(fun);
            end
        end

        %------------------------------------------------------------------
        function removeRadialCoeffsButtonListener(this)
            parent = this.Parent;
            isStereo = parent.isStereoAppMode();
            if isStereo
                this.StandardOptionsSection.RadialCoeffsButton1.ValueChangedFcn = [];
                this.StandardOptionsSection.RadialCoeffsButton2.ValueChangedFcn  = [];
            else
                this.StandardOptionsPanel.removeRadialCoeffsButtonListener();
            end
        end
    end % end of public methods
    %----------------------------------------------------------------------
    % protected methods
    %----------------------------------------------------------------------
    methods(Access=protected)
        function protectOnDelete(~, fHandle, varargin)
            %protectOnDelete is a wrapper around any callback that protects
            % against deletion of the App window while the callback is
            % being processed. Wrap your callback with this method to avoid
            % command line errors being thrown in such circumstances.
            %
            % See the configureDisplays method in VideoLabelingTool for
            % example.
            try
                fHandle(varargin{:});
            catch ME
                if strcmp(ME.identifier, 'MATLAB:class:InvalidHandle')
                    % Do NOT throw error messages on the command line if the
                    % App has been deleted while processing a callback.
                    return
                end
            end
        end
    end %end of protected methods
    
    %----------------------------------------------------------------------
    % Private methods
    %----------------------------------------------------------------------
    methods (Access=private)
        %------------------------------------------------------------------
        function updateCalibrateButtonState(this, session)
            if session.IsFixedIntrinsics
                areIntrinsicsLoaded = ~isempty(session.StereoIntrinsics1) && ...
                    ~isempty(session.StereoIntrinsics2);
            else
                areIntrinsicsLoaded = true;
            end
            this.CalibrateSection.IsButtonEnabled = ...
                session.HasEnoughBoards && ~session.CanExport && ...
                areIntrinsicsLoaded;
            
            if this.CalibrateSection.IsButtonEnabled
                this.CalibrateSection.setToolTip(...
                    'vision:caltool:EnabledCalibrateToolTip');
            else
                if session.CanExport
                    this.CalibrateSection.setToolTip(...
                        'vision:caltool:DisabledCurrentCalibrationToolTip');
                else
                    this.CalibrateSection.setToolTip(...
                        'vision:caltool:DisabledCalibrateToolTip');
                end
            end
        end
        
        
        %------------------------------------------------------------------
        function updateIntrinsicsButtonState(this, session)
            if session.IsFixedIntrinsics
                this.IntrinsicsOptionsSection.IsFixedIntrinsicsSelected = true;
                this.StandardOptionsSection.setAllButtonsEnabled(false);
                this.OptimizationOptionsSection.disableButton();
            else
                this.IntrinsicsOptionsSection.IsFixedIntrinsicsSelected = false;
                % Enable all the camera model and optimization options
                % panels: disable numcoeffs buttons if specified in
                % Optimization options.
                this.StandardOptionsSection.setAllButtonsEnabled(true);
                if isfield(session.OptimizationOptions, 'InitialDistortion')
                    if isempty(session.OptimizationOptions.InitialDistortion)
                        this.enableNumRadialCoefficients();
                    else
                        numCoeffs = numel(session.OptimizationOptions.InitialDistortion);
                        this.disableNumRadialCoefficients(numCoeffs);
                    end
                end
                this.OptimizationOptionsSection.enableButton();
            end
        end
        
        %------------------------------------------------------------------
        function updateViewUndistortButtonState(this, session)
            parent = this.Parent;
            
            if parent.isStereoAppMode()
                % In the stereoCameraCalibrator, the ViewUndistortedSection is
                % a Toolstrip section which updates differntly.It has the
                % callback to viewShowRectifiedSection
                if isCalibrated(session) && session.CanExport
                    this.ViewUndistortSection.enableButton();
                else
                    this.ViewUndistortSection.disableButton();
                    this.ViewUndistortSection.setupButton(false);
                end
            else
                if isCalibrated(session) && session.CanExport
                    if session.IsFisheyeModel
                        this.ViewUndistortSection.setUndistortButtonEnabled(true);
                        isOriginal = this.ViewUndistortSection.isUndistortButtonPressed();
                        if isOriginal
                            this.ViewUndistortSection.updateScaleFactorControls(true);
                        else
                            this.ViewUndistortSection.updateScaleFactorControls(false);
                        end
                    else % standard model
                        this.ViewUndistortSection.setAllButtonsEnabled(false);
                        this.ViewUndistortSection.setUndistortButtonEnabled(true);
                    end
                else
                    % Only at App initialization.
                    this.ViewUndistortSection.setAllButtonsEnabled(false);
                    % unpress the View Undistorted button.
                    this.ViewUndistortSection.setupUndistortButton(false);
                    this.ViewUndistortSection.resetAppliedScaleFactor();
                end
            end
        end
        
        %------------------------------------------------------------------
        function updateExportButtonState(this, session)
            if ~isempty(session.PatternSet)
                updateExportButtonState(this.ExportSection, session.CanExport, session.PatternSet.PatternDetector);
            else
                updateExportButtonState(this.ExportSection, session.CanExport);
            end
            
            if session.IsFisheyeModel
                this.ExportSection.setText(getString(message(('vision:caltool:ExportFisheyeButton'))));
            else
                this.ExportSection.setText(getString(message(('vision:caltool:ExportButton'))));
            end
            
            if this.ExportSection.IsButtonEnabled
                setToolTip(this.ExportSection, ...
                    'vision:caltool:EnabledExportToolTip');
            else
                setToolTip(this.ExportSection, ...
                    'vision:caltool:DisabledExportToolTip');
            end
        end
        
        %------------------------------------------------------------------
        function createWidgets(this, isStereo)
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % Tool-strip sections
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            if ~isStereo
                this.FileSection = vision.internal.calibration.tool.section.FileSection;
            else
                this.FileSection = vision.internal.calibration.tool.section.FileSectionStereo;
            end
            this.Tab.add(this.FileSection.Section);
            
            this.ZoomSection = vision.internal.calibration.tool.section.ZoomSection;
            this.Tab.add(this.ZoomSection.Section);
            
            this.LayoutSection = vision.internal.calibration.tool.section.LayoutSection;
            this.Tab.add(this.LayoutSection.Section);
            
            if ~isStereo
                this.CameraOptionsSection = vision.internal.calibration.tool.section.CalibrationModelSection;
                this.Tab.add(this.CameraOptionsSection.Section);
            else
                this.IntrinsicsOptionsSection = vision.internal.calibration.tool.section.IntrinsicsOptionsSection;
                this.Tab.add(this.IntrinsicsOptionsSection.Section);
                
                this.StandardOptionsSection = vision.internal.calibration.tool.section.StandardOptionsSectionStereo;
                this.Tab.add(this.StandardOptionsSection.Section);
                
                this.OptimizationOptionsSection = vision.internal.calibration.tool.section.OptimizationOptionsSection;
                this.Tab.add(this.OptimizationOptionsSection.Section);
            end
            
            
            this.CalibrateSection = vision.internal.calibration.tool.section.CalibrateSection;
            this.Tab.add(this.CalibrateSection.Section);
            
            if ~isStereo
                this.ViewUndistortSection = vision.internal.calibration.tool.section.ViewUndistortedSection;
            else
                this.ViewUndistortSection = vision.internal.calibration.tool.section.ViewShowRectifiedSection;
            end
            this.Tab.add(this.ViewUndistortSection.Section);
            
            this.ExportSection = vision.internal.calibration.tool.section.ExportSection;
            this.Tab.add(this.ExportSection.Section);
        end
        %------------------------------------------------------------------
        function installListeners(this,isStereo)
            this.installListenersFileSection(isStereo);
            this.installListenersLayoutSection();
            this.installListenersCalibrateSection();
            this.installListenersExportSection();
            this.installListenersCameraOptionsSection(isStereo);
            this.installListenersIntrinsicsOptionsSection();
            this.installListenersStandardOptionsSectionStereo();
            this.installListenersOptimizationOptionsSection();
            this.installListenersViewUndistortSection(isStereo);
        end
        %------------------------------------------------------------------
        function installListenersFileSection(this, isStereo)
            this.FileSection.NewSessionButton.ButtonPushedFcn = ...
                @(es,ed)newSession(this.Parent);
            
            this.FileSection.OpenSessionButton.ButtonPushedFcn = ...
                @(es,ed)openSession(this.Parent);
            this.FileSection.SaveSessionButton.ButtonPushedFcn = ...
                @(es,ed)saveSession(this.Parent);
            this.FileSection.SaveSession.ItemPushedFcn = ...
                @(es,ed)saveSession(this.Parent);
            this.FileSection.SaveasSession.ItemPushedFcn = ...
                @(es,ed)saveSessionAs(this.Parent);
            if isStereo
                this.FileSection.AddImagesButton.ButtonPushedFcn = ...
                    @(es,ed)addImages(this.Parent);
            else
                this.FileSection.AddImagesButton.ButtonPushedFcn = @(es,ed)addImages(this.Parent);
                this.FileSection.AddImagesFromFile.ItemPushedFcn = @(es,ed)addImages(this.Parent);
                this.FileSection.AddImagesFromCamera.ItemPushedFcn = @(es,ed)addImagesFromCamera(this.Parent);
            end
        end
        % -----------------------------------------------------------------
        
        % The listeners for the ZoomSection are controlled by the
        % CameraCalibrationTool class.
        
        %------------------------------------------------------------------
        function installListenersLayoutSection(this)
            this.LayoutSection.DefaultLayoutButton.ButtonPushedFcn = ...
                @(es,ed)layout(this.Parent);
        end
        %------------------------------------------------------------------
        function installListenersCameraOptionsSection(this,isStereo)
            if ~isStereo
                funOpts = @(es,ed)doCameraOptions(this.Parent);
                this.CameraOptionsSection.OptionButton.DynamicPopupFcn = funOpts;
                this.CameraOptionsSection.StandardRadioButton.ValueChangedFcn = @(es,ed)cameraModelChanged(this.Parent, false);
                this.CameraOptionsSection.FisheyeRadioButton.ValueChangedFcn = @(es,ed)cameraModelChanged(this.Parent, true);
                
                % Create panels for popups
                fun = @(es,ed)cameraModelChanged(this.Parent);
                funInit = @(es,ed)doInitializationOptions(this.Parent);
                this.FisheyeOptionsPanel.AlignmentButton.ValueChangedFcn = fun;
                
                this.FisheyeOptionsPanel = vision.internal.calibration.webTool.FisheyeParameterPanel(fun);
                showInitButton = true;
                this.StandardOptionsPanel = vision.internal.calibration.webTool.StandardOptionsPanel(showInitButton, fun, funInit);
                if this.CameraOptionsSection.IsFisheyeSelected
                    this.CameraOptionsSection.OptionButton.Popup = this.FisheyeOptionsPanel.Panel;
                else
                    this.CameraOptionsSection.OptionButton.Popup = this.StandardOptionsPanel.PopupList;
                end
            end
        end
        
        %------------------------------------------------------------------
        function installListenersIntrinsicsOptionsSection(this)
            btnPressFcn = @(es, ed)toggleIntrinsicsMode(this.Parent);
            loadBtnPress = @(es, ed)doLoadIntrinsics(this.Parent);
            this.IntrinsicsOptionsSection.UseFixedIntrinsicsRadioBtn.ValueChangedFcn = btnPressFcn;
            this.IntrinsicsOptionsSection.ComputeIntrinsicsRadioBtn.ValueChangedFcn  = btnPressFcn;
            this.IntrinsicsOptionsSection.LoadIntrinsicsButton.ButtonPushedFcn = loadBtnPress;
        end
        
        %------------------------------------------------------------------
        function installListenersStandardOptionsSectionStereo(this)
            fun = @(es,ed)cameraModelChanged(this.Parent);
            this.StandardOptionsSection.RadialCoeffsButton1.ValueChangedFcn = fun;
            this.StandardOptionsSection.RadialCoeffsButton2.ValueChangedFcn  = fun;
            this.StandardOptionsSection.CheckSkewButton.ValueChangedFcn = fun;
            this.StandardOptionsSection.CheckTangentialButton.ValueChangedFcn = fun;
        end
        
        %------------------------------------------------------------------
        function installListenersOptimizationOptionsSection(this)
            fun = @(es,ed)doInitializationOptions(this.Parent);
            this.OptimizationOptionsSection.OptimizationOptionsButton.ButtonPushedFcn = fun;
        end
        %------------------------------------------------------------------
        function installListenersCalibrateSection(this)
            fun = @(es, ed)this.protectOnDelete(@(obj)calibrate(this.Parent));
            this.CalibrateSection.CalibrateButton.ButtonPushedFcn = fun;
        end
        %------------------------------------------------------------------
        function installListenersViewUndistortSection(this, isStereo)
            import vision.internal.calibration.tool.*;
            
            if ~isStereo
                funUndistort = @(es,ed)imageViewChanged(this.Parent);
                this.ViewUndistortSection.ShowUndistortToggleButton.ValueChangedFcn = funUndistort;
                % listener for scale factor spinner button
                addlistener(this.ViewUndistortSection.ScaleFactorSpinner,'ValueChanged',@(es,ed)scaleFactorApplied(this.Parent));
            else
                % For the Stereo App: single 'Show Rectified' button
                funUndistort = @(es,ed)rectifiedViewChanged(this.Parent);
                this.ViewUndistortSection.ShowRectifiedButton.ValueChangedFcn = funUndistort;
            end
        end
        
        %------------------------------------------------------------------
        function installListenersExportSection(this)
            this.ExportSection.ExportButton.ButtonPushedFcn = ...
                @(es,ed)export(this.Parent);
            this.ExportSection.ExportParametersToWS.ItemPushedFcn = ...
                @(es,ed)export(this.Parent);
            this.ExportSection.GenerateMATLABScript.ItemPushedFcn = ...
                @(es,ed)generateCode(this.Parent);
        end
        %------------------------------------------------------------------
    end % end of private methods
    
end % end of class definition
