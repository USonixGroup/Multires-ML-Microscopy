% CaptureSection Encapsulates Capture tool strip section
%
% This class represents the Capture section used in Image Capture tab of
% Camera Calibrator App
% 
% CaptureSection properties:
% 
%  CaptureButton - Button to capture Images. Invoking the
%                  button will start capturing images based on the
%                  capture interval and number of images to capture.
%                  Can be stopped earlier.

% Copyright 2019-2023 The MathWorks, Inc.

classdef CaptureSection < vision.internal.uitools.NewToolStripSection
    properties
        CaptureButton
    end
    
    methods
        %------------------------------------------------------------------
        function this = CaptureSection()
            this.createSection();
            this.layoutSection();
        end
        %---------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:caltool:CaptureSection'));
            tag = 'CaptureSection';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %---------------------------------------------------------------------
        function layoutSection(this)
            this.addCaptureButton();
            
            col1 = this.addColumn();
            col1.add(this.CaptureButton);
            
        end
        %-----------------------------------------------------------------------
        function addCaptureButton(this)
            import matlab.ui.internal.toolstrip.*
            
            CaptureLabel = 'vision:caltool:StartCaptureButton';
            tag = 'btnCapture';
            icon = Icon('playControl');
            this.CaptureButton = this.createButton(icon,CaptureLabel, tag);
            
        end
    end
end
