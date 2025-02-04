%ImageSequenceSource Loads the image sequences to GroundTruth Labeler App
%   ImageSequenceSource is a MultiSignalSource that is used to load an
%   image sequence as a signal in Ground Truth Labeler App. 
%   
%   Example: Create a image sequence source
%   ---------------------------------------
%
%   % Specify path for the image sequence directory.
%   imageSeqDir = fullfile(toolboxdir('driving'), 'drivingdata', ...
%       'roadSequence');
% 
%   % Load time stamps corresponding to the image sequence
%   load(fullfile(imageSeqDir,'timeStamps.mat'))
% 
%   % Create a image sequence source
%   sourceName = imageSeqDir;
%   sourceParams = struct();
%   sourceParams.Timestamps = timeStamps;
% 
%   imseqSource = vision.labeler.loading.ImageSequenceSource();
%   imseqSource.loadSource(sourceName, sourceParams);
% 
%   % Read the first frame from the image sequence
%   signalName = imseqSource.SignalName;
%   I = imseqSource.readFrame(signalName, 1);
% 
%   % Display the frame
%   figure, imshow(I)
%
%   See also vision.labeler.loading.MultiSignalSource

%   Copyright 2019-2023 The MathWorks, Inc.

classdef ImageSequenceSource < vision.labeler.loading.MultiSignalSource
    
    properties
        Name = string(vision.getMessage('vision:labeler:ImageSequenceDisplayName'))
        
        Description = string(vision.getMessage('vision:labeler:ImageSequenceDes'))
    end
    
    properties (Access = private)
        %Imds Image datastore
        Imds
    end
    
    % Panel Properties
    properties (Access = private)
        Panel
        FolderPathText
        FolderPathBox
        FolderBrowseButton
        TimeStampsText
        TimeStampsDropDown
        TimeStampsNote
        
        FolderPathTextPos
        FolderPathBoxPos
        FolderBrowseButtonPos
        TimeStampsTxtPos
        TimeStampsDropDownPos 
        TimeStampsNotePos
    end
    
    
    methods
        %------------------------------------------------------------------
        % Load Panel Methods
        %------------------------------------------------------------------          
        function customizeLoadPanel(this, panel)
            this.Panel = panel;
            
            computePositions(this);
            
            addUIComponents(this);           
        end
        
        function [sourceName, sourceParams] = getLoadPanelData(this)
            sourceParams = struct();
            sourceName = string(this.FolderPathBox.Value);   
        end
        
        %------------------------------------------------------------------
        % Load Source
        %------------------------------------------------------------------           
        function loadSource(this, sourceName, sourceParams)
           
            % Load file
            this.Imds = imageDatastore(sourceName);
            vision.internal.labeler.validation.validateImageSequence(this.Imds)
            
            % Populate timestamps
            
            if isempty(this.Timestamp)
                if isfield(sourceParams, 'Timestamps')
                    setTimestamps(this, sourceParams.Timestamps);
                else
                    this.Timestamp = {seconds(0:1:numel(this.Imds.Files)-1)'};
                end
            else
                if ~iscell(this.Timestamp)
                    this.Timestamp = {this.Timestamp};
                end
            end
            
            import vision.internal.labeler.validation.*
            checkImageSequenceAndTimestampsAgreement(this.Imds,this.Timestamp{1});
            
            % Populate signal names and types
            [~, folderName, ~] = fileparts(sourceName);
            
            this.SignalName = makeValidName(this, string(folderName), "imageSequence_");
            this.SignalType = vision.labeler.loading.SignalType.Image;
            
            this.SourceName = sourceName;
            this.SourceParams = sourceParams;
        end   
        
        %------------------------------------------------------------------
        % Read frame
        %------------------------------------------------------------------          
        function frame = readFrame(this, signalName, index)
            if ~strcmpi(signalName, this.SignalName)
                frame = [];
            else
                frame = readimage(this.Imds, index);
            end            
        end

    end 
    
    methods (Access = private)

        %------------------------------------------------------------------
        % Position calculations
        %------------------------------------------------------------------ 
        function computePositions(this)
            
            padding = 5;
            widgetHeight = 25;
            
            folderNameTxtX = 10;
            folderNameWidth = 90;
            folderNameBoxWidth = 300;
            folderNameButtonWidth = 70;
            
            timeStampPadding = 100;
            timeStampTxtWidth = 90;
            timeStampDropwDownWidth = 150;
            timeStampNoteWidth = 250;
            
            yPosition = 50;
            folderNameBoxX = folderNameTxtX + folderNameWidth + padding;
            folderNameButtonX = folderNameBoxX + folderNameBoxWidth + padding;
            timeStampTxtX = folderNameButtonX + folderNameButtonWidth + timeStampPadding;
            timeStampDropDownX = timeStampTxtX + timeStampTxtWidth + padding;            
           
            this.FolderPathTextPos     = [folderNameTxtX (yPosition - (widgetHeight/4)) folderNameWidth widgetHeight+4];
            this.FolderPathBoxPos      = [folderNameBoxX yPosition folderNameBoxWidth widgetHeight];
            this.FolderBrowseButtonPos = [folderNameButtonX yPosition folderNameButtonWidth widgetHeight];
            
            this.TimeStampsTxtPos      = [timeStampTxtX yPosition timeStampTxtWidth widgetHeight];
            this.TimeStampsDropDownPos = [timeStampDropDownX yPosition timeStampDropwDownWidth widgetHeight];  
            this.TimeStampsNotePos     = [timeStampTxtX yPosition-34 timeStampNoteWidth + 10 widgetHeight+7]; 
        end
        
        %------------------------------------------------------------------
        % UI Components
        %------------------------------------------------------------------ 
        function addUIComponents(this)
            
            addFileBrowseComponents(this);
            
            addTimeStampsComponents(this);
            
        end
        
        function addFileBrowseComponents(this)
            
                this.FolderPathText = uilabel('Parent', this.Panel,...
                    'Text', vision.getMessage('vision:labeler:FolderName'),...
                    'Position', this.FolderPathTextPos,...
                    'Tag', 'fileText');
                
                this.FolderPathBox = uieditfield('Parent', this.Panel,...
                    'Value', '',...
                    'Position', this.FolderPathBoxPos,...
                    'Tag', 'fileEditBox',...
                    'ValueChangedFcn',@(src,evt)this.updateFolderPathBoxValue(src,evt));
                
                this.FolderBrowseButton = uibutton('Parent', this.Panel,...
                    'Text', vision.getMessage('vision:labeler:Browse'),...
                    'Position', this.FolderBrowseButtonPos,...
                    'ButtonPushedFcn', @this.browseButtonCallback,...
                    'Tag', 'browseBtn');                
           
        end
        
        function addTimeStampsComponents(this)
           
            this.TimeStampsText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:Timestamps'),...
                'Position', this.TimeStampsTxtPos,...
                'Tag', 'timeStampTxt');
    
            this.TimeStampsDropDown = uidropdown('Parent', this.Panel,...
                'Items', {vision.getMessage('vision:labeler:UseDefault'), vision.getMessage('vision:labeler:FromWorkspace')},...
                'Position', this.TimeStampsDropDownPos,...
                'ValueChangedFcn', @this.timeStampsDropDownCallback,...
                'Tag', 'timeStampSourceSelectList');       
    
            this.TimeStampsNote = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:DefaultTimestampsIS'),...
                'Position', this.TimeStampsNotePos,...
                'Tag', 'timeStampNote');                 
            
        end        
        
        %------------------------------------------------------------------
        % Callbacks
        %------------------------------------------------------------------ 
        
        function updateFolderPathBoxValue(this,src,~)
            this.FolderPathBox.Value = src.Value;
        end

        function browseButtonCallback(this, ~, ~)
            
            persistent folderPath
            
            if isempty(folderPath)
                folderPath = pwd;
            end
            
            disableBrowseButton(this);
            enableBrowse = onCleanup(@()set(this.FolderBrowseButton, 'Enable', 'on'));                   
            
            newFolderName = uigetdir(folderPath,...
                vision.getMessage('vision:labeler:SelectFolder'));
            
            hFig = ancestor(this.Panel, 'figure');
            figure(hFig);  
            
            if newFolderName ~= 0 
                folderPath = newFolderName;
            else
                return;
            end
            
            this.FolderPathBox.Value = folderPath;
            
           
        end
        
        function timeStampsDropDownCallback(this, ~, ~)
            
            selection = this.TimeStampsDropDown.Value;
           
            if strcmp(selection, vision.getMessage('vision:labeler:UseDefault'))
                if ~isempty(this.Timestamp)
                    this.Timestamp = [];
                end
                this.TimeStampsNote.Visible = 'on';
            elseif strcmp(selection, vision.getMessage('vision:labeler:FromWorkspace'))
                this.TimeStampsNote.Visible = 'off';
                variableTypes = {'duration'};
                variableDisp =  {'duration'};
                [timestamps,~,isCanceled] = vision.internal.uitools.getVariablesFromWS(variableTypes, variableDisp);
                
                if isCanceled
                    return;
                end
                
                setTimestamps(this, timestamps);
            else
                % Do nothing
            end
        end
      
    end
    
    methods (Hidden)
        function isPending = checkPendingChanges(this)
            %checkPendingChanges Checks for pending changes in load panel
            %   checkPendingChanges returns a boolean flag isPending, if
            %   there are any pending changes in the load panel.
            isPending = ~isempty(this.FolderPathBox.Value);                
                       
        end
        
        function disableBrowseButton(this)
            this.FolderBrowseButton.Enable = 'off';
        end    
        
    end        
end