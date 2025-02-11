% CloseSection Encapsulates Close tool strip section
%
% This class represents the Close section used in Image Capture tab of
% Camera Calibrator App
%
% CloseSection properties:
% 
%  CloseButton - Button to close capturing Images. Invoking the
%               button will save whatever images captured and
%               returns back to Calibration tab

% Copyright 2019-2023 The MathWorks, Inc.

classdef CloseSection < vision.internal.uitools.NewToolStripSection
    properties
        CloseButton
    end
    
    methods
        %------------------------------------------------------------------
        function this = CloseSection()
            this.createSection();
            this.layoutSection();
        end
        %---------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:CloseSection'));
            tag = 'CloseSection';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %---------------------------------------------------------------------
        function layoutSection(this)
            this.addCloseButton();
            
            col1 = this.addColumn();
            col1.add(this.CloseButton);
            
        end
        %-----------------------------------------------------------------------
        function addCloseButton(this)
            import matlab.ui.internal.toolstrip.*
            
            CloseLabel = 'vision:caltool:CloseImageCaptureButton';
            tag = 'btnClose';
            icon = Icon('close');
            this.CloseButton = this.createButton(icon,CloseLabel, tag);
            toolTipID = 'vision:caltool:CloseButtonToolTip';
            this.setToolTipText(this.CloseButton, toolTipID);
        end
    end
end