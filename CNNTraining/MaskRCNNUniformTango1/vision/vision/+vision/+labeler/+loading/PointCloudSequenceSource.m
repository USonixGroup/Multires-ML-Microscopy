%PointCloudSequenceSource Loads the point cloud sequences to GroundTruth Labeler App
%   PointCloudSequenceSource is a MultiSignalSource that is used to load a
%   point cloud sequence as a signal in Ground Truth Labeler App. 
%
%   See also vision.labeler.loading.MultiSignalSource,
%   vision.labeler.loading.ImageSequenceSource.
%
%   Example: Create a point cloud sequence source
%   ---------------------------------------------
%   
%   % Specify the path for point cloud sequence
%   pcSeqDir = fullfile(toolboxdir('driving'), 'drivingdata',...
%   'lidarSequence');
%
%   % Load time stamps corresponding to the sequence
%   load(fullfile(pcSeqDir, 'timestamps.mat'));
% 
%   % Create a point cloud sequence source
%   sourceName = pcSeqDir;
%   sourceParams = struct();
%   sourceParams.Timestamps = timestamps;
% 
%   pcseqSource = vision.labeler.loading.PointCloudSequenceSource();
%   pcseqSource.loadSource(sourceName, sourceParams);
% 
%   % Read the fifth frame from the sequence
%   signalName = pcseqSource.SignalName;
%   pc = pcseqSource.readFrame(signalName, 5);
% 
%   % Display the frame
%   figure, pcshow(pc)

%   Copyright 2019-2023 The MathWorks, Inc.

classdef PointCloudSequenceSource < vision.labeler.loading.MultiSignalSource
    
    properties
        Name = string(vision.getMessage('vision:labeler:PointCloudSequenceDisplayName'))
        
        Description = string(vision.getMessage('vision:labeler:PointCloudSequenceDes'))
    end
    
    properties (Access = private)
        %Pcds Point Cloud datastore
        Pcds
    end
    
    % Panel Properties
    properties (Access = private)
        Panel
        FolderPathText
        FolderPathBox
        FolderTextBox
        FolderBrowseButton
        TimeStampsText
        TimeStampsDropDown
        TimeStampsNote
        
        FolderPathTextPos
        FolderPathBoxPos
        FolderTextPos
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
            ext = {'.pcd', '.ply'};
            this.Pcds = fileDatastore(sourceName,'ReadFcn', @pcread, 'FileExtensions', ext);
            
            % Populate timestamps
            
            if isempty(this.Timestamp)
                if isfield(sourceParams, 'Timestamps')
                    setTimestamps(this, sourceParams.Timestamps);
                else
                    this.Timestamp = {seconds(0:1:numel(this.Pcds.Files)-1)'};
                end
            else
                if ~iscell(this.Timestamp)
                    this.Timestamp = {this.Timestamp};
                end
            end
            
            import vision.internal.labeler.validation.*
            checkPointCloudSequenceAndTimestampsAgreement(this.Pcds,this.Timestamp{1});
            
            % Populate signal names and types
            [~, folderName, ~] = fileparts(sourceName);
            
            this.SignalName = makeValidName(this, string(folderName), "pointcloudSequence_");
            this.SignalType = vision.labeler.loading.SignalType.PointCloud;
            
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
                frame = pcread(this.Pcds.Files{index});
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
            
            this.FolderPathTextPos     = [folderNameTxtX (yPosition - (widgetHeight/4)) folderNameWidth widgetHeight];
            this.FolderPathBoxPos      = [folderNameBoxX yPosition folderNameBoxWidth widgetHeight];
            this.FolderTextPos         = [folderNameBoxX yPosition-30 folderNameBoxWidth widgetHeight];
            this.FolderBrowseButtonPos = [folderNameButtonX yPosition folderNameButtonWidth widgetHeight];
            
            this.TimeStampsTxtPos      = [timeStampTxtX yPosition timeStampTxtWidth widgetHeight];
            this.TimeStampsDropDownPos    = [timeStampDropDownX yPosition timeStampDropwDownWidth widgetHeight];
            this.TimeStampsNotePos     = [timeStampTxtX yPosition-30 timeStampNoteWidth+10 widgetHeight+4]; 
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
                'Tag', 'fileEditBox');
            
            this.FolderTextBox = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:SupportedFileType'),...
                'Position', this.FolderTextPos,...
                'Tag', 'fileText');
            
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
                'Text', vision.getMessage('vision:labeler:DefaultTimestampsPC'),...
                'Position', this.TimeStampsNotePos,...
                'Tag', 'timeStampNote');
                
        end
        
        %------------------------------------------------------------------
        % Callbacks
        %------------------------------------------------------------------
        
        function browseButtonCallback(this, ~, ~)
            
            persistent folderPath
            
            if isempty(folderPath)
                folderPath = pwd;
            end
            
            disableBrowseButton(this);
            enableBrowse = onCleanup(@()set(this.FolderBrowseButton, 'Enable', 'on'));                  
            
            newFolderName = uigetdir(folderPath,...
                vision.getMessage('vision:labeler:SelectPointCloudFolder'));
            
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
