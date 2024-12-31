% ViewUndistortedSection Encapsulates view settings tool strip Section
%
%   This class represents the view settings panel used in
%   Camera Calibrator App. It is NOT used in the stereoCameraCalibrator App.
%
%   ViewUndistorteSection creates a tool strip panel
%   containing the view options controls.
%
%   ViewUndistortedSection properties:
%
%       ShowUndistortToggleButton - Button to toggle undistorted view.
%       ScaleFactorButton         - Button to open tearaway controlling scale.
%
%   ViewUndistortedPanel methods:
%
%       setupUndistortButton        - set or unset 'view undistort' button.
%       setUndistortButtonEnabled   - enable/disable 'view undistort' button.
%       setScaleFactorButtonEnabled - enable/disable scale factor button.
%       isUndistortButtonPressed    - return state of 'view undistort' button.
%       setAllButtonsEnabled        - enable/disable all buttons.
%       setUndistortButtonText      - set text label for undistort button.
%       getScaleFactorButton        - return the TSButton for scale factor.

% Copyright 2017-2023 The MathWorks, Inc.

classdef ViewUndistortedSection < vision.internal.uitools.NewToolStripSection
    properties
        ShowUndistortToggleButton
          
        ScaleFactorLabel
        
        ScaleFactorSpinner
        AppliedScaleFactor
        
        ScaleFactorPanel
        
    end
    
    properties (Constant)
        LowValue = 0.001;
        HighValue = 5; % Suitable for most applications.
        DefaultScaleFactor = 1;
    end
    
    methods
        function this = ViewUndistortedSection()
            this.createSection();
            this.layoutSection();
           
        end
        
        %------------------------------------------------------------------
        function btn = getScaleFactorButton(this)
            btn = this.ScaleFactorButton;
        end
        
        %------------------------------------------------------------------
        function setUndistortButtonText(this, text)
            this.ShowUndistortToggleButton.Text = text;
        end
        
        %------------------------------------------------------------------
        function setUndistortButtonToolTip(this, toolTipID)
            this.setToolTipText(this.ShowUndistortToggleButton, toolTipID);
        end
        
        %------------------------------------------------------------------
        function setUndistortButtonIcon(this, option)
            if(strcmp(option,'original'))
                undistortBtnIcon =  matlab.ui.internal.toolstrip.Icon('checkerboard');
                this.ShowUndistortToggleButton.Icon = undistortBtnIcon;
            elseif(strcmp(option,'distorted'))
                undistortBtnIcon =  matlab.ui.internal.toolstrip.Icon('fisheyeCheckerboard');
                this.ShowUndistortToggleButton.Icon = undistortBtnIcon;
            end
        end
        %------------------------------------------------------------------
        function setupUndistortButton(this, state)
        % setUndistortButton Set or unset UndistortButton to state.
            this.ShowUndistortToggleButton.Value = state;
        end

        %------------------------------------------------------------------
        function TF = isUndistortButtonPressed(this)
        % isUndistortButtonPressed Check if the undistort image button is
        % pressed or not. If pressed, current display is the undistorted
        % image. Else, currently displaying original image.
            TF = this.ShowUndistortToggleButton.Value;
        end

        %------------------------------------------------------------------
        function setUndistortButtonEnabled(this, state)
        % setUndistortButtonEnabled Enable or disable undistortbutton
        %   specifically. This is used in the standard camera case. state
        %   is a logical scalar.
            this.ShowUndistortToggleButton.Enabled = state;
        end

        %------------------------------------------------------------------
        function setAllButtonsEnabled(this, state)
        % setAllButtonsEnabled Enable or disable all panel controls
        %   setAllButtonsEnabled(panel, state) enables or disables all
        %   controls of the panel. panel is a ToolStripPanel object. state
        %   is a logical scalar.
            this.ShowUndistortToggleButton.Enabled = state;
            this.ScaleFactorSpinner.Enabled = state;
            this.ScaleFactorLabel.Enabled = state;
        end

       %------------------------------------------------------------------
        function updateScaleFactorControls(this,status)
            this.ScaleFactorLabel.Enabled = status;
            this.ScaleFactorSpinner.Enabled= status;
        end

        %------------------------------------------------------------------
        function val = getScaleFactorValue(this)
            val = this.ScaleFactorSpinner.Value;
        end
        
        %------------------------------------------------------------------
        function val = getAppliedScaleFactorValue(this)
            val = this.AppliedScaleFactor;
        end   
        %------------------------------------------------------------------
        function setScaleFactorSpinnerValue(this)
            this.ScaleFactorSpinner.Value = this.AppliedScaleFactor.Value;
        end

        %------------------------------------------------------------------
        function updateAppliedScaleFactor(this)
           this.AppliedScaleFactor =  getScaleFactorValue(this);
          
        end
        
        %------------------------------------------------------------------
        function resetAppliedScaleFactor(this)
            this.AppliedScaleFactor = this.DefaultScaleFactor;
            this.ScaleFactorSpinner.Value = this.AppliedScaleFactor;
        end
        %------------------------------------------------------------------
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:UndistortSection'));
            tag = 'undistortOpts';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %-------------------------------------------------------------------
        function layoutSection(this)
            this.addButtons();
            
            colViewUndistort = this.addColumn();
            colViewUndistort.add(this.ShowUndistortToggleButton);
            this.ScaleFactorPanel = matlab.ui.internal.toolstrip.Panel;
            scaleFactorLabelLayout = this.ScaleFactorPanel.addColumn();
            scaleFactorLabelLayout.add(this.ScaleFactorLabel);
            scaleFactorSpinnerLayout = this.ScaleFactorPanel.addColumn('Width',50);
            scaleFactorSpinnerLayout.add(this.ScaleFactorSpinner);
            colViewUndistort.add(this.ScaleFactorPanel);
        end
        %------------------------------------------------------------------
        function createUndistortButton(this)
            undistortBtnIcon = matlab.ui.internal.toolstrip.Icon('fisheyeCheckerboard');
            toolTipID = 'vision:caltool:UndistortUndistortedToolTip';
            titleID = 'vision:caltool:UndistortUndistorted';
            this.ShowUndistortToggleButton = this.createToggleButton(undistortBtnIcon, titleID, ...
                'UndistortBtn');
            this.setToolTipText(this.ShowUndistortToggleButton, toolTipID);
        end
        
        
        %------------------------------------------------------------------
        function createScaleFactorButton(this)
            
            titleId = 'vision:caltool:ScaleFactorBtn';
            toolTipID = 'vision:caltool:FisheyeScaleFactorToolTip';
            % Icon is not required here as per UX recommendation
            this.ScaleFactorLabel = this.createLabel(titleId);
            this.ScaleFactorLabel.Description = getString(message(toolTipID));
            this.ScaleFactorLabel.Tag = 'fisheyeScaleFactorLabel';

            
            this.ScaleFactorSpinner = matlab.ui.internal.toolstrip.Spinner([this.LowValue, this.HighValue], this.DefaultScaleFactor);
            this.ScaleFactorSpinner.Tag = 'scaleFactorTextField';
            this.ScaleFactorSpinner.NumberFormat = 'double';
            this.ScaleFactorSpinner.StepSize = 0.1;
            
        end
        %------------------------------------------------------------------
        function addButtons(this)
            % Initialize undistort button with 'Show Undistorted'
            
            this.createUndistortButton();
            this.createScaleFactorButton();
        end
        
    end
end
