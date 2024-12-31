% CalibrationModelSection Encapsulates camera model choice tool strip
% section
%
%   This class represents the camera model choice panel used in
%   Camera Calibrator App. It is NOT used in the stereoCameraCalibrator App.
%
%   CalibrationModelSection creates a tool strip section containing the 
%   camera model choice controls.
%
%   CalibrationModelSection properties:
%
%       StandardButton - A radio button representing the standard model
%       FisheyeButton  - A radio button representing the fisheye model
%
%   CalibrationModelSection methods:
%
%       setAllButtonsEnabled - enable or disable all the panel controls
%       selectStandardModel  - select the standard camera model
%       selectFisheyeModel   - select the fisheye camera model

% Copyright 2017-2023 The MathWorks, Inc.

classdef CalibrationModelSection < vision.internal.uitools.NewToolStripSection
    properties
        CameraModelLabel
        StandardRadioButton
        FisheyeRadioButton
        
        OptionButton
        CalibrationChoiceButtonGroup
    end
    
    properties(Dependent)
        IsFisheyeSelected
    end
    
    methods
        
        %------------------------------------------------------------------
        function this = CalibrationModelSection()
            this.createSection();
            this.layoutSection();
        end
        %------------------------------------------------------------------
        function TF = get.IsFisheyeSelected(this)
            TF = this.FisheyeRadioButton.Value;
        end
        
        %------------------------------------------------------------------
        function set.IsFisheyeSelected(this, boolVal)         
            this.FisheyeRadioButton.Value = boolVal;
            this.StandardRadioButton.Value = ~boolVal;
        end
        
        %------------------------------------------------------------------
        function setAllButtonsEnabled(this, state)
            % setAllButtonsEnabled Enable or disable all panel controls
            %   setAllButtonsEnabled(panel, state) enables or disables all
            %   controls of the panel. panel is a ToolStripPanel object. state
            %   is a logical scalar.
            this.StandardRadioButton.Enabled = state;
            this.FisheyeRadioButton.Enabled = state;
            this.OptionButton.Enabled = state;
        end
        
        %------------------------------------------------------------------
        function button = getOptionButton(this)
            % getOptionButton Get option button handle for tearaway.
            button = this.OptionButton;
        end
        
        %------------------------------------------------------------------
        function setOptionButtonText(this, text)
            % setText Set the button text
            %   setText modifies the text on the button.
            this.OptionButton.Text = text;
        end
        %------------------------------------------------------------------
        function selectStandardModel(this)
            this.IsFisheyeSelected = false;
        end
        %------------------------------------------------------------------
        function selectFisheyeModel(this)
            this.IsFisheyeSelected = true;
        end
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:ModelSection'));
            tag = 'modelOptions';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %------------------------------------------------------------------
        function layoutSection(this)
            this.addModelRadioButtons();
            this.addOptionButton();
            
            colCameraModel = this.addColumn();
            colCameraModel.add(this.CameraModelLabel);
            colCameraModel.add(this.StandardRadioButton);
            colCameraModel.add(this.FisheyeRadioButton);
            
            colOptions = this.addColumn();
            colOptions.add(this.OptionButton);
        end
        %------------------------------------------------------------------
        function addModelRadioButtons(this)
            this.CameraModelLabel = this.createLabel('vision:caltool:TypeButtonGroupHeading');
            this.createModelRadioButtons();
        end
        
        %------------------------------------------------------------------
        function addOptionButton(this)
            titleID = 'vision:caltool:OptionButton';
            icon = matlab.ui.internal.toolstrip.Icon('settings');
            tag = 'cameraModelOptionButton';
            
            this.OptionButton = this.createDropDownButton(icon, titleID, tag);
            toolTipID = 'vision:caltool:OpenModelOptionsToolTip';
            this.setToolTipText(this.OptionButton, toolTipID);
        end
        
        %------------------------------------------------------------------
        function createModelRadioButtons(this)
            titleId = 'vision:caltool:StandardButton';
            toolTipId = 'vision:caltool:StandardModelToolTip';
            tag = 'btnStandard';
            group1 = matlab.ui.internal.toolstrip.ButtonGroup;
            this.StandardRadioButton = this.createRadioButton(titleId, tag, toolTipId, group1);
            
            titleId = 'vision:caltool:FisheyeButton';
            toolTipId = 'vision:caltool:FisheyeModelToolTip';
            tag = 'btnFisheye';
            group2 = matlab.ui.internal.toolstrip.ButtonGroup;
            this.FisheyeRadioButton = this.createRadioButton(titleId, tag, toolTipId, group2);
        end
    end
end
