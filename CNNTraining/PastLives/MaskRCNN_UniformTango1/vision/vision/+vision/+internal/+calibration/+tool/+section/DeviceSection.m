% DeviceSection Encapsulates Device tool strip section
%
% This class represents the Device section used in Image Capture tab of
% Camera Calibrator App
% 
% DeviceSection properties:
% 
%  PropertiesButton - Button to change the properties of images
%                     to be captured. Invoking the button will open
%                     the panel through which Properties have to
%                     be set before capturing the images.
%
%  DevicePanel      - combination of DeviceLabel and Device Combobox to
%                     specify the device (webcam by default)

% Copyright 2019-2023 The MathWorks, Inc.

classdef DeviceSection < vision.internal.uitools.NewToolStripSection
    
    
    properties(Access = public)
        DeviceLabel
        DeviceComboBox
        DevicePanel
        PropertiesButton
    end
    
    methods
        %------------------------------------------------------------------
        function this = DeviceSection()
            this.createSection();
            this.layoutSection();
        end
        %---------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:DeviceSection'));
            tag = 'DeviceSection';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %---------------------------------------------------------------------
        function layoutSection(this)
            % create a layout and add the buttons within the columns
            this.addCameraButton();
            this.addPropertiesButton();
            
            col1 = this.addColumn();
            DevPanel1 = this.DevicePanel.addColumn();
            DevPanel1.add(this.DeviceLabel)
            DevPanel2 = this.DevicePanel.addColumn();
            DevPanel2.add(matlab.ui.internal.toolstrip.Label(' '));
            DevPanel3 = this.DevicePanel.addColumn();
            DevPanel3.add(this.DeviceComboBox)
            col1.add(this.DevicePanel);
            col1.add(this.PropertiesButton)
        end
        %-----------------------------------------------------------------------
        function addCameraButton(this)
            this.DeviceLabel = this.createLabel('vision:caltool:DeviceDropDown');
            tag = 'dropDownDevice';
            toolTipID = 'vision:caltool:EnabledCameraDropDownToolTip';
            % Create ComboBox
            this.DeviceComboBox = this.createDropDown({''},tag,toolTipID);
            
            % Create the panel.
            this.DevicePanel = matlab.ui.internal.toolstrip.Panel;
        end
        %-----------------------------------------------------------------------
        function addPropertiesButton(this)
            import matlab.ui.internal.toolstrip.*
            
            PropertiesLabel = 'vision:caltool:PropertiesButton';
            PropertiesIcon = Icon('settings');
            tag = 'btnProperties';
            toolTipID = 'vision:caltool:EnabledCameraPropertiesToolTip';
            % Create Button
            this.PropertiesButton = this.createButton(PropertiesIcon,PropertiesLabel, tag);
            this.setToolTipText(this.PropertiesButton, toolTipID);
        end
    end
    
end