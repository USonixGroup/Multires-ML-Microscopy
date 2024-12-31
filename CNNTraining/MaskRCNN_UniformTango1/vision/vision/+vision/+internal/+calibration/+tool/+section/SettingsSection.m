% SettingsSection Encapsulates Settings tool strip section
%
% This class represents the Settings section used in
% Image Capture tab of Camera Calibrator App
% 
% SettingsSection properties:
% 
%  BrowseButton            - Button to browse the directory in which the
%                            captured images has to be stored.
%  CaptureIntervalPanel    - combination of Edit field and Slider to
%                            mention the interval in which the
%                            image should be captured.
%  NumImagesToCapturePanel - combination of Edit field and Slider to have
%                            a count of number of images
%                            to be captured.

% Copyright 2019-2023 The MathWorks, Inc.

classdef SettingsSection < vision.internal.uitools.NewToolStripSection
    properties
        SaveLocationLabel
        SaveLocationEdit
        CurrentSaveLocation
        BrowseButton
        
        CaptureIntervalLabel
        CaptureIntervalEdit
        CaptureIntervalSlider
        CaptureIntervalPanel
        
        NumImagesCaptureLabel
        NumImagesCaptureEdit
        NumImagesCaptureSlider
        NumImagesToCapturePanel
    end
    
    
    methods
        %------------------------------------------------------------------
        function this = SettingsSection()
            this.createSection();
            this.layoutSection();
        end
        %---------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:SettingsSection'));
            tag = 'SettingsSection';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %---------------------------------------------------------------------
        function layoutSection(this)
            this.addSaveLocationTextField();
            this.addBrowseButton();
            this.addCaptureInterval();
            this.addNumImgToCapture();
            
            colPanel1 = this.addColumn();
            colPanel1.add(this.SaveLocationLabel);
            colPanel2 = this.addColumn('Width', 300);
            colPanel2.add(this.SaveLocationEdit)
            colPanel3 = this.addColumn();
            colPanel3.add(this.BrowseButton)
            
            captureIntPanel1 = this.CaptureIntervalPanel.addColumn('Width', 40);
            captureIntPanel1.add(this.CaptureIntervalEdit)
            captureIntPanel2 = this.CaptureIntervalPanel.addColumn();
            captureIntPanel2.add(this.CaptureIntervalSlider)
            
            colPanel1.add(this.CaptureIntervalLabel);
            colPanel2.add(this.CaptureIntervalPanel);
            
            NumImgToCapPanel1 = this.NumImagesToCapturePanel.addColumn('Width', 40);
            NumImgToCapPanel1.add(this.NumImagesCaptureEdit)
            NumImgToCapPanel2 = this.NumImagesToCapturePanel.addColumn();
            NumImgToCapPanel2.add(this.NumImagesCaptureSlider)
            
            colPanel1.add(this.NumImagesCaptureLabel);
            colPanel2.add(this.NumImagesToCapturePanel);
            
        end
        %-----------------------------------------------------------------------
        function addSaveLocationTextField(this)
            this.SaveLocationLabel = this.createLabel('vision:caltool:SaveLocationEdit');
            tf = vision.internal.calibration.webTool.ImageCaptureTab.hasWritePermissions('',pwd, false);
            if tf
                startDir = pwd;
            else
                startDir = '';
            end
            this.CurrentSaveLocation = startDir;
            this.SaveLocationEdit = matlab.ui.internal.toolstrip.EditField(this.CurrentSaveLocation);
            this.SaveLocationEdit.Tag = 'editSaveLocation';
            this.SaveLocationEdit.Description = vision.getMessage('vision:caltool:EnabledSaveLocationToolTip');
            
        end
        %-----------------------------------------------------------------------
        function addBrowseButton(this)
            import matlab.ui.internal.toolstrip.*
            
            BrowseLabel = 'vision:caltool:BrowseButton';
            BrowseIcon = Icon('openFolder');
            tag = 'btnBrowse';
            toolTipID = 'vision:caltool:BrowseButtonToolTip';
            this.BrowseButton = this.createButton(BrowseIcon,BrowseLabel, tag);
            this.setToolTipText(this.BrowseButton, toolTipID);
        end
        %-----------------------------------------------------------------------
        function addCaptureInterval(this)
            this.CaptureIntervalLabel = this.createLabel('vision:caltool:CaptureIntervalEdit');
            
            % Create slider
            defValue = 5;
            range = [vision.internal.calibration.webTool.ImageCaptureTab.MinInterval...
                vision.internal.calibration.webTool.ImageCaptureTab.MaxInterval];
            tag = 'sliderCaptureInterval';
            toolTipId = 'vision:caltool:EnabledCaptureIntervalToolTip';
            this.CaptureIntervalSlider = this.createSlider(range, defValue, tag, toolTipId);
            this.CaptureIntervalSlider.Compact = true;
            
            this.CaptureIntervalEdit = matlab.ui.internal.toolstrip.EditField(num2str(defValue));
            this.CaptureIntervalEdit.Tag = 'editCaptureInterval';
            % Create the panel.
            this.CaptureIntervalPanel = matlab.ui.internal.toolstrip.Panel;
        end
        %-----------------------------------------------------------------------
        function addNumImgToCapture(this)
            defValue = 20;
            this.NumImagesCaptureLabel = this.createLabel('vision:caltool:ImagesToCaptureEdit');
            this.NumImagesCaptureEdit = matlab.ui.internal.toolstrip.EditField(num2str(defValue));
            this.NumImagesCaptureEdit.Tag = 'editNumImagesCapture';
            this.NumImagesCaptureEdit.Description = vision.getMessage('vision:caltool:EnabledNumImagesCaptureToolTip');
            
            % Create slider component for num images to capture control.
            
            range = [vision.internal.calibration.webTool.ImageCaptureTab.MinImages...
                vision.internal.calibration.webTool.ImageCaptureTab.MaxImages];
            toolTipId = 'vision:caltool:EnabledNumImagesCaptureToolTip';
            tag = 'sliderNumImagesCapture';
            this.NumImagesCaptureSlider = this.createSlider(range, defValue, tag, toolTipId);
            this.NumImagesCaptureSlider.Compact = true;
            
            % Create the panel.
            this.NumImagesToCapturePanel = matlab.ui.internal.toolstrip.Panel;
            
        end
    end
end