% CalibrateSection Encapsulates Calibrate tool strip section
%
% This class represents the Calibrate section used in both Camera calibrator
% and Stereo camera calibrator App
%
% CalibrateSection properties:
% 
%  Calibrate Button  - Button to Calibrate the Images which are added.
%                      Invoking the button will Calibrate the images
%                      based on the Camera Model, incase of Camera
%                      calibratror App and standard Options along
%                      with Optimization options, incase of stereo
%                      Camera Calibrator.

% Copyright 2019-2023 The MathWorks, Inc.

classdef CalibrateSection < vision.internal.uitools.NewToolStripSection
    properties
        CalibrateButton
    end
    
    properties(Dependent)
        IsButtonEnabled
    end
    
    methods
        function set.IsButtonEnabled(this, IsEnabled)
            this.CalibrateButton.Enabled = IsEnabled;
        end
        
        function IsEnabled = get.IsButtonEnabled(this)
            IsEnabled = this.CalibrateButton.Enabled;
        end
    end
    
    methods
        %------------------------------------------------------------------
        function this = CalibrateSection()
            this.createSection();
            this.layoutSection();
        end
        
        %---------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:CalibrateSection'));
            tag = 'secCalibrate';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
            
        end
        %---------------------------------------------------------------------
        function layoutSection(this)
            
            this.addCalibrateButton();
            
            colCalibrate = this.addColumn();
            colCalibrate.add(this.CalibrateButton);
            
        end
        %---------------------------------------------------------------------
        function addCalibrateButton(this)
            import matlab.ui.internal.toolstrip.*;
            
            % New Session Button
            CalibrateTitleId = 'vision:caltool:CalibrateButton';
            CalibrateLayoutIcon = Icon('playControl');
            CalibrateLayoutTag = 'btnCalibrate';
            this.CalibrateButton = this.createButton(CalibrateLayoutIcon, ...
                CalibrateTitleId, CalibrateLayoutTag);
            
        end
        %------------------------------------------------------------------
        function setToolTip(this, toolTipId)
            % setToolTip Set the button tool tip. 
            %   toolTipId is the message catalog id of the tool tip string.
            this.setToolTipText(this.CalibrateButton, toolTipId);
        end
    end
end