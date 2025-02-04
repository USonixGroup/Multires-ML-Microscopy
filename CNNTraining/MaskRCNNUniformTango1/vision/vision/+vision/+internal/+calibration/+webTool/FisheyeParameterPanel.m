% FisheyeParameterPanel Encapsulates fisheye parameter control tool strip panel
%
%   This class represents the fisheye control panel used in
%   Camera Calibrator App. It is NOT used in the stereoCameraCalibrator App.
%
%   panel = FisheyeParameterPanel(fun) creates a tool strip panel
%   containing the calibration options controls. fun is a handle to the
%   callback function.
%
%   FisheyeParameterPanel properties:
%
%       AlignmentButton - check box to control alignment estimation
%
%   FisheyeParameterPanel methods:
%
%       setAllButtonsEnabled - enable or disable the panel controls
%       enableFisheyeButtons - enable check box
%       enableFisheyeButtons - disable check box

% Copyright 2017-2020 The MathWorks, Inc.

classdef FisheyeParameterPanel < handle
    properties (Access=private)
        AlignmentButton
    end
    
    properties(Access = public)
        Panel
    end
    
    properties(Dependent)
        % CameraModel A struct containing the calibration options
        % corresponding to the current state of the panel. The struct
        % contains the following fields:
        %   EstimateAlignment - true or false
        CameraModel
    end
    
    methods
        function this = FisheyeParameterPanel(fun)
            this.createPanel();
            this.addButtons();
            this.addCallback(fun);
        end
        
        %------------------------------------------------------------------
        % Gets the camera model options
        %------------------------------------------------------------------
        function cameraModel = get.CameraModel(this)
            if this.AlignmentButton.Value
                cameraModel.EstimateAlignment = true;
            else
                cameraModel.EstimateAlignment = false;
            end
        end
        
        %------------------------------------------------------------------
        function set.CameraModel(this, cameraModel)
            if cameraModel.EstimateAlignment
                this.AlignmentButton.Value = true;
            else
                this.AlignmentButton.Value = false;
            end
        end
        
        %------------------------------------------------------------------
        function setAllButtonsEnabled(this, state)
            % setAllButtonsEnabled Enable or disable all panel controls
            %   setAllButtonsEnabled(panel, state) enables or disables all
            %   controls of the panel. panel is a ToolStripPanel object. state
            %   is a logical scalar.
            this.AlignmentButton.Enabled = state;
        end
        
        %------------------------------------------------------------------
        function enableFisheyeButtons(this)
            this.AlignmentButton.Enabled = true;
        end
        
        %------------------------------------------------------------------
        function disableFisheyeButtons(this)
            this.AlignmentButton.Enabled = false;
        end
    end
    
    methods(Access = protected)
        %------------------------------------------------------------------
        function  createPanel(this)
            this.Panel = matlab.ui.internal.toolstrip.PopupList();
            tag = 'panelFisheyeParams';
            this.Panel.Tag = tag;
        end
        
        %------------------------------------------------------------------
        function addButtons(this)
            this.createAlignmentCheckBox();
            add(this.Panel, this.AlignmentButton);
        end
        
        %------------------------------------------------------------------
        function createAlignmentCheckBox(this)
            titleId = 'vision:caltool:AlignmentButton';
            toolTipId = 'vision:caltool:EstimateAlignmentToolTip';
            this.AlignmentButton = matlab.ui.internal.toolstrip.ListItemWithCheckBox(...
                getString(message(titleId)));
            this.AlignmentButton.Tag = 'AlignBtn';
            this.AlignmentButton.Description = getString(message(toolTipId));
        end
        
        %------------------------------------------------------------------
        function addCallback(this, fun)
            addButtonCallback(this.AlignmentButton, fun);
        end
    end
end

%--------------------------------------------------------------------------
function addButtonCallback(button, fun)
addlistener(button, 'ValueChanged', fun);
end
