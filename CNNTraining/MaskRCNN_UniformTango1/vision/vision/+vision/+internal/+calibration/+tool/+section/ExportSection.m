% ExportSection Encapsulates the Export tool strip section
%
% This class represents the Export section used in both Camera Calibrator
% and Stereo Camera Calibrator App
%
% ExportButton - Button to export camera parameters after calibration
%                is done. Camera parameters can be either exported to
%                workspace or a MATLAB script can be generated.

% Copyright 2019-2024 The MathWorks, Inc.

classdef ExportSection < vision.internal.uitools.NewToolStripSection
    properties
        ExportButton
        
        ExportParametersToWS
        GenerateMATLABScript
    end
    properties(Dependent)
        IsButtonEnabled
    end
    
    methods
        %------------------------------------------------------------------
        function this = ExportSection()
            this.createSection();
            this.layoutSection();
        end
        %---------------------------------------------------------------------
        function createSection(this)
            title = getString(message('vision:uitools:ExportSection'));
            tag = 'ExportSection';
            this.Section = matlab.ui.internal.toolstrip.Section(title);
            this.Section.Tag = tag;
        end
        %---------------------------------------------------------------------
        function layoutSection(this)
            
            this.addExportButton();
            
            colExport = this.addColumn();
            colExport.add(this.ExportButton);
            
        end
        %-----------------------------------------------------------------------
        function addExportButton(this)
            import matlab.ui.internal.toolstrip.*;
     
            exportIcon      = Icon('greenCheck');
            exportTitleID   = 'vision:caltool:ExportButton';
            exportTag       = 'btnExport';
            this.ExportButton = this.createSplitButton(exportIcon, exportTitleID,...
                exportTag);
            
            % To Workspace
            
            text = vision.getMessage('vision:caltool:ExportParametersPopup');
            icon = Icon('greenCheck');
            this.ExportParametersToWS = ListItem(text,icon);
            this.ExportParametersToWS.ShowDescription = false;
            this.ExportParametersToWS.Tag = 'itemExportParametersToWS';
            
            % Generate MATLAB script
            text = vision.getMessage('vision:caltool:GenerateScriptPopup');
            icon = Icon('generateScript_matlab');
            this.GenerateMATLABScript = ListItem(text,icon);
            this.GenerateMATLABScript.ShowDescription = false;
            this.GenerateMATLABScript.Tag = 'itemGenerateMATLABScript';
            
            % Construct definitions popup
            defsPopup = matlab.ui.internal.toolstrip.PopupList('IconSize',16);
            defsPopup.add(this.ExportParametersToWS);
            defsPopup.add(this.GenerateMATLABScript);
            this.ExportButton.Popup = defsPopup;
        end
        %------------------------------------------------------------------
        function setText(this, text)
            % setText Set the button text
            % setText modifies the text on the button.
            this.ExportButton.Text = text;
        end
        %------------------------------------------------------------------
        function setToolTip(this, toolTipId)
            % setToolTip Set the button tool tip. 
            %   toolTipId is the message catalog id of the tool tip string.
            
            this.setToolTipText(this.ExportButton, toolTipId);
        end
        
        %------------------------------------------------------------------
        function updateExportButtonState(this, isEnabled, varargin)
            this.ExportButton.Enabled = isEnabled;
            this.GenerateMATLABScript.Enabled = true;
            if nargin > 2
                patternDetector = varargin{1};
                if ~isNativePatternDetector(patternDetector)
                    this.GenerateMATLABScript.Enabled = false;
                end
            end
            
            function tf = isNativePatternDetector(patternDetector)
            
                import vision.internal.calibration.tool.*
                stereoDetectorsList = getNativeDetectorsList("stereo");
                monocularDetectorsList = getNativeDetectorsList("monocular");
                nativePatternDetectorsList = [stereoDetectorsList, ...
                                              monocularDetectorsList];
            
                className = class(patternDetector);
                tf = any(strcmp(nativePatternDetectorsList,className));
            end
        end
    end
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function isEnabled = get.IsButtonEnabled(this)
            isEnabled = this.ExportButton.Enabled;
        end
    end
end
