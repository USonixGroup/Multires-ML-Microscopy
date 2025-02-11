% ViewShowRectifiedSection Encapsulates view tool strip section for
% StereoCameraCalibrator App 
%
% ViewShowRectifiedSection creates a tool
%  strip section containing the Button to show rectified result
%
% ViewShowRectifiedSection properties:
%
%  ShowRectifiedButton   - button to show rectified  

% Copyright 2019-2023 The MathWorks, Inc.

classdef ViewShowRectifiedSection < vision.internal.uitools.NewToolStripSection
    properties
        ShowRectifiedButton
    end
    
    methods
        function this = ViewShowRectifiedSection()
            this.createSection();
            this.layoutSection();
        end
        %------------------------------------------------------------------
        function TF = isButtonPressed(this)
            % isButtonPressed Check if the undistort image button is
            % pressed or not.
            TF = this.ShowRectifiedButton.Value;
        end
        
        %------------------------------------------------------------------
        function setupButton(this, state)
            % setUndistortButton Set or unset Button to state.
            this.ShowRectifiedButton.Value = state;
        end
        %-------------------------------------------------------------------
        function enableButton(this)
            % enableButton Enable the button
            this.ShowRectifiedButton.Enabled = true;
        end
        
        %------------------------------------------------------------------
        function disableButton(this)
            % disableButton Disable the button
            this.ShowRectifiedButton.Enabled = false;
        end
        %------------------------------------------------------------------
        function setButtonToolTip(this, toolTipID)
            this.setToolTipText(this.ShowRectifiedButton, toolTipID);
        end
        
        %------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:UndistortSection'));
            tag = 'viewRectified';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %------------------------------------------------------------------
        function layoutSection(this)
            this.addShowRectifiedButton();
            colViewRectified = this.addColumn();
            colViewRectified.add(this.ShowRectifiedButton);
        end
        %------------------------------------------------------------------
        function addShowRectifiedButton(this)
            showRectifiedBtnIcon = matlab.ui.internal.toolstrip.Icon('rectifiedDisplay');
            nameId   = 'vision:caltool:RectifiedButton';
            tag = 'btnRectified';
            toolTipId = 'vision:caltool:ShowRectifiedStereoImagesToolTip';
            this.ShowRectifiedButton = this.createToggleButton(showRectifiedBtnIcon, nameId, tag);
            this.setToolTipText(this.ShowRectifiedButton,toolTipId);
        end
        %-------------------------------------------------------------------
    end
    
end