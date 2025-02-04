% FileSection Encapsulates the File tool strip section
%
% This class represents the File section used in both Camera Calibrator 
% and Stereo Camera Calibrator App
% 
%   NewSessionButton    - Button to open a new session.While invoked, if
%                         there's an existing session it will ask for
%                         saving else it will directly open a new session.
%   OpenSessionButton   - Button to open an already existing session which
%                         was saved earlier.
%   AddImagesButton     - Button to add images from file for calibration.

% Copyright 2019-2023 The MathWorks, Inc.

classdef FileSectionStereo< vision.internal.uitools.NewToolStripSection
    properties
        NewSessionButton
        
        OpenSessionButton
        
        SaveSessionButton
        SaveSession
        SaveasSession
        
        AddImagesButton
    end
    methods
        %------------------------------------------------------------------
        function this = FileSectionStereo()
            this.createSection();
            this.layoutSection();
        end
        
        %------------------------------------------------------------------
        function createSection(this)
            fileSectionTitle = getString(message('vision:uitools:FileSection'));
            fileSectionTag   = 'sectionFile';
            this.Section = matlab.ui.internal.toolstrip.Section(fileSectionTitle);
            this.Section.Tag = fileSectionTag;
        end
        
        %-------------------------------------------------------------------
        function layoutSection(this)
            this.addNewSessionButton();
            this.addOpenSessionButton();
            this.addSaveSessionButton();
            this.addImagesButton();
            
            colNewSession = this.addColumn();
            colNewSession.add(this.NewSessionButton);
            
            colOpenSession = this.addColumn();
            colOpenSession.add(this.OpenSessionButton);
            
            colSaveSession = this.addColumn();
            colSaveSession.add(this.SaveSessionButton);
            
            colAddImages = this.addColumn();
            colAddImages.add(this.AddImagesButton);
        end
        %------------------------------------------------------------------
        function addNewSessionButton(this)
            icon = matlab.ui.internal.toolstrip.Icon('new');
            titleId = 'vision:uitools:NewSessionButton';
            tag = 'btnNewSession';
            this.NewSessionButton = this.createButton(icon, titleId, tag);
            NewSessionToolTip  = 'vision:caltool:NewSessionToolTip';
            this.setToolTipText(this.NewSessionButton, NewSessionToolTip);
        end
        %------------------------------------------------------------------
        function addOpenSessionButton(this)
            icon = matlab.ui.internal.toolstrip.Icon('openFolder');
            titleId = 'vision:uitools:OpenSessionButton';
            tag = 'btnOpenSession';
            this.OpenSessionButton = this.createButton(icon, titleId, tag);
            OpenSessionToolTip = 'vision:caltool:OpenSessionToolTip';
            this.setToolTipText(this.OpenSessionButton, OpenSessionToolTip);
        end
        %------------------------------------------------------------------
        function addSaveSessionButton(this)
            icon = matlab.ui.internal.toolstrip.Icon('saved');
            titleId = 'vision:uitools:SaveSessionButton';
            tag = 'btnSaveSession';
            this.SaveSessionButton = this.createSplitButton(icon, titleId, tag);
            SaveSessionToolTip  = 'vision:caltool:SaveSessionToolTip';
            this.setToolTipText(this.SaveSessionButton, SaveSessionToolTip);
            
            % Popup Option Entry : Save
            icon = matlab.ui.internal.toolstrip.Icon('saved');
            saveSessionText = vision.getMessage('vision:uitools:SaveSessionOption');
            this.SaveSession = matlab.ui.internal.toolstrip.ListItem(saveSessionText, icon);
            this.SaveSession.ShowDescription = false;
            this.SaveSession.Tag = 'calibSaveSession';
            
            % Popup Option Entry :Saveas
            icon = matlab.ui.internal.toolstrip.Icon('saveAs');
            saveasSessionText = vision.getMessage('vision:uitools:SaveSessionAsOption');
            this.SaveasSession = matlab.ui.internal.toolstrip.ListItem(saveasSessionText, icon);
            this.SaveasSession.ShowDescription = false;
            this.SaveasSession.Tag = 'calibSaveSessionAs';
            
            % Construct Popup
            savePopup = matlab.ui.internal.toolstrip.PopupList('IconSize',16);
            savePopup.add(this.SaveSession);
            savePopup.add(this.SaveasSession);
            this.SaveSessionButton.Popup = savePopup;
        end
        
        %------------------------------------------------------------------
        function addImagesButton(this)

            icon = matlab.ui.internal.toolstrip.Icon('import_data');
            titleId = 'vision:uitools:AddImagesButton';
            tag = 'btnAddImages';
            this.AddImagesButton = this.createButton(icon, titleId, tag);
            AddImagesToolTip   = 'vision:caltool:AddImagesToolTip';
            this.setToolTipText(this.AddImagesButton,AddImagesToolTip);
        end
    end
    % Update Button States
    methods
        %------------------------------------------------------------------
        function disableAddImagesButton(this)
            this.AddImagesButton.Enabled = false;
        end
        
        %------------------------------------------------------------------
        function enableAddImagesButton(this)
            this.AddImagesButton.Enabled = true;
        end
        
        %------------------------------------------------------------------
        function disableSaveButton(this)
            this.SaveSessionButton.Enabled = false;
        end
        
        %------------------------------------------------------------------
        function enableSaveButton(this)
            this.SaveSessionButton.Enabled = true;
        end
        
    end
    
    
end
