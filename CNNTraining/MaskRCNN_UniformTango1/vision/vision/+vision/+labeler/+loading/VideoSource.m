%VideoSource Loads video signals to Ground Truth Labeler App
%   VideoSource is a MultiSignalSource that is used to load video signals
%   to Ground Truth Labele App in Automated Driving Toolbox (TM)
%
%   Example: Create a video source
%   ------------------------------
%   % Create the video source and load the video
%   sourceName = 'caltech_cordova1.avi';
%   sourceParams = [];
%
%   vidSource = vision.labeler.loading.VideoSource();
%   vidSource.loadSource(sourceName, sourceParams);
%
%   % Read the first frame from the video
%   signalName = vidSource.SignalName;
%   I = vidSource.readFrame(signalName, 1);
%
%   % Display the frame
%   figure, imshow(I)
%
%   See also vision.labeler.loading.MultiSignalSource

%   Copyright 2019-2023 The MathWorks, Inc.

classdef VideoSource < vision.labeler.loading.MultiSignalSource
    
    properties 
        %Name String or character vector specifying the name of the loader
        Name = string(vision.getMessage('vision:labeler:VideoDisplayName'))
        
        %Description String or character vector describing the loader
        Description = string(vision.getMessage('vision:labeler:VideoDes'))
    end
    
    % Reader properties
    properties (Access = private)
        %FileReader A video file reader
        FileReader

        %UseVideoURI If VideoURI is to be used for rendering frames
        UseVideoURI = false
    end
    
    % Panel properties
    properties (Access = private)
        Panel
        FileNameText
        FileNameBox
        FileBrowseButton
        TimeStampsText
        TimeStampsDropDown
        
        FileNameTxtPos
        FileNameBoxPos
        FileNameButtonPos
        TimeStampsTxtPos
        TimeStampsDropDownPos
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
            sourceName = this.FileNameBox.Value;
            sourceParams = struct();
        end
        
        %------------------------------------------------------------------
        % Load source
        %------------------------------------------------------------------  
        function loadSource(this, sourceName, sourceParams)
            % Give the sourceName as a char input
            if ~ischar(sourceName)
                sourceName = char(sourceName);
            end
            % Load file
            this.FileReader = matlab.internal.VideoReader(sourceName);
            
            % Populate timestamps  
            if isempty(this.Timestamp)
                if isfield(sourceParams, 'Timestamps')
                    setTimestamps(this, sourceParams.Timestamps);
                else
                    this.Timestamp = {this.FileReader.Timestamps};
                end
            else
                if ~iscell(this.Timestamp)
                    this.Timestamp = {this.Timestamp};
                end
            end
                
            if this.FileReader.NumFrames ~= numel(this.Timestamp{1})
                error(message('vision:labeler:VideoTimeStampsError'));
            end
                
            % Populate signal names and types
            [~, fileName, ~] = fileparts(sourceName);
            
            this.SignalName = makeValidName(this, fileName, "video_");
            this.SignalType = vision.labeler.loading.SignalType.Image;
            
            this.SourceName = fullfile(this.FileReader.Path, this.FileReader.Name);
            this.SourceParams = sourceParams;
        end        

        %------------------------------------------------------------------
        % Read frame
        %------------------------------------------------------------------            
        function frame = readFrame(this, signalName, index)
            if ~strcmpi(signalName, this.SignalName)
                frame = [];
            else
                try
                    if this.UseVideoURI
                        % For VideoURI frame rendering, we do not load the
                        % actual video frame, but use the timestamp and
                        % video file path to render onscreen via the TextureURI
                        % feature.
                        frame.Data = fullfile(this.FileReader.Path, this.FileReader.Name);
                        frame.Timestamp = this.Timestamp{1}(index);
                        frame.Height = this.FileReader.Height;
                        frame.Width = this.FileReader.Width;
                    else
                        % Default video frame loading
                    frame = readFrameAtPosition(this.FileReader, index);
                    frame = frame.Data;
                    end
                catch
                    % so that it won't error out when nested callbacks are
                    % called at last frame for reading the next frame
                    frame = [];
                end
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
            
            fileNameTxtX = 10;
            fileNameWidth = 70;
            fileNameBoxWidth = 300;
            fileNameButtonWidth = 70;
            
            timeStampPadding = 100;
            timeStampTxtWidth = 90;
            timeStampDropwDownWidth = 150;
            
            yPosition = 50;
            filenameBoxX = fileNameTxtX + fileNameWidth + padding;
            fileNameButtonX = filenameBoxX + fileNameBoxWidth + padding;
            timeStampTxtX = fileNameButtonX + fileNameButtonWidth + timeStampPadding;
            timeStampDropDownX = timeStampTxtX + timeStampTxtWidth + padding;
            
            this.FileNameTxtPos     = [fileNameTxtX (yPosition - (widgetHeight/4)) fileNameWidth widgetHeight];
            this.FileNameBoxPos     = [filenameBoxX yPosition fileNameBoxWidth widgetHeight];
            this.FileNameButtonPos  = [fileNameButtonX yPosition fileNameButtonWidth widgetHeight];
            
            this.TimeStampsTxtPos   = [timeStampTxtX (yPosition) timeStampTxtWidth widgetHeight];
            this.TimeStampsDropDownPos = [timeStampDropDownX yPosition timeStampDropwDownWidth widgetHeight];
        end
        
        %------------------------------------------------------------------
        % UI Components
        %------------------------------------------------------------------ 
        function addUIComponents(this)
            
            addFileBrowseComponents(this);
            
            addTimeStampsComponents(this);
            
        end
        
        function addFileBrowseComponents(this)
               
            this.FileNameText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:FileName'),...
                'Position', this.FileNameTxtPos,...
                'Tag', 'fileText');
            
            this.FileNameBox = uieditfield('Parent', this.Panel,...
                'Value', '',...
                'Position', this.FileNameBoxPos,...
                'Tag', 'fileEditBox');
            
            this.FileBrowseButton = uibutton('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:Browse'),...
                'Position', this.FileNameButtonPos,...
                'ButtonPushedFcn', @this.browseButtonCallback,...
                'Tag', 'browseBtn');
            
        end
        
        function addTimeStampsComponents(this)
            
            this.TimeStampsText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:Timestamps'),...
                'Position', this.TimeStampsTxtPos,...
                'Tag', 'timeStampTxt');
            
            this.TimeStampsDropDown = uidropdown('Parent', this.Panel,...
                'Items', {vision.getMessage('vision:labeler:FromFile'), vision.getMessage('vision:labeler:FromWorkspace')},...
                'Position', this.TimeStampsDropDownPos,...
                'ValueChangedFcn', @this.timeStampsDropDownCallback,...
                'Tag', 'timeStampSourceSelectList');
            
        end          
        %------------------------------------------------------------------
        % Callbacks
        %------------------------------------------------------------------ 
        
        function browseButtonCallback(this, ~, ~)
            
            disableBrowseButton(this);
            enableBrowse = onCleanup(@()set(this.FileBrowseButton, 'Enable', 'on'));
            
            [fileName,userCanceled] = vision.internal.videoLabeler.videogetfile();
            
            hFig = ancestor(this.Panel, 'figure');
            figure(hFig);
            
            if userCanceled
                return;
            end
            
            this.FileNameBox.Value = fileName;
          
        end
        
        function timeStampsDropDownCallback(this, ~, ~)
            
            selection = this.TimeStampsDropDown.Value;
            
            if strcmp(selection, vision.getMessage('vision:labeler:FromFile'))
                if ~isempty(this.Timestamp)
                    this.Timestamp = [];
                end
            elseif strcmp(selection, vision.getMessage('vision:labeler:FromWorkspace'))
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
            isPending = ~isempty(this.FileNameBox.Value);                
                        
        end
        
        function disableBrowseButton(this)
            this.FileBrowseButton.Enable = 'off';
        end    


        function info = getFileReaderInfo(this)
            %getFileReaderInfo Get file reader information specific to this source.
            % Used by writeVideoScenes.
            info.VideoFormat = this.FileReader.VideoFormat;
            if isprop(this.FileReader, 'Colormap')
                info.Colormap = this.FileReader.Colormap; 
            end
        end

        function setUseVideoURI(this, flag)
            this.UseVideoURI = flag;
        end

        function flag = getUseVideoURI(this)
            flag = this.UseVideoURI;
        end
        
    end
end
