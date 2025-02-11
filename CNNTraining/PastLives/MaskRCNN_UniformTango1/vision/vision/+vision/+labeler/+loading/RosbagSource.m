%RosbagSource Interface for loading the rosbag to GroundTruth Labeler App
%
%   RosbagSource specifies an interface to load signals from rosbag into
%   the Ground Truth Labeler app. Using this class requires that you have 
%   ROS Toolbox (TM). Signals corresponding to the following
%   message types will be loaded.
%
%       sensor_msgs/Image
%       sensor_msgs/CompressedImage
%       sensor_msgs/PointCloud2
%
%  Copyright 2019-2023 The Mathworks, Inc.

classdef RosbagSource < vision.labeler.loading.MultiSignalSource
    
    % Base class properties
    properties
        Name = string(vision.getMessage('vision:labeler:RosbagDisplayName'))
        
        Description = string(vision.getMessage('vision:labeler:RosbagDes'))
    end
    
    % Properties only required by this class
    properties (Access = private)
        BagSelect
        
        ValidBagSelect
        
        TopicSelectArray
    end
    
    properties (Constant, Access = private)
        ValidMessageTypes = ["sensor_msgs/Image",...
                             "sensor_msgs/PointCloud2",...
                             "sensor_msgs/CompressedImage"...
                             ];        
        ValidSignalTypes = [vision.labeler.loading.SignalType.Image,...
                            vision.labeler.loading.SignalType.PointCloud,...
                            vision.labeler.loading.SignalType.Image...
                            ];
    end
    
    % Panel properties
    properties (Access = private)
        Panel
        FileNameText
        FileNameBox
        FileBrowseButton
        NoteText
        
        FileNameTxtPos
        FileNameBoxPos
        FileNameButtonPos
        NoteTextPos
    end
    
    methods
        
        %------------------------------------------------------------------
        % Load Panel Methods
        %------------------------------------------------------------------         
        function customizeLoadPanel(this, panel)
            
            vision.internal.requiresROSToolbox(mfilename);
            
            this.Panel = panel;
            
            computePositions(this);
            
            addUIComponents(this);

        end
        
        function [sourceName, sourceParams] = getLoadPanelData(this)
            sourceParams = struct;     
            sourceName = string(this.FileNameBox.Value);         
        end
        
        function loadSource(this, sourceName, sourceParams)
            
            % Load rosbag
            vision.internal.requiresROSToolbox(mfilename);
            
            this.BagSelect = rosbag(sourceName);
            
            % Choose signals to read from
            this.ValidBagSelect = select(this.BagSelect, 'MessageType',...
                this.ValidMessageTypes);
            
            validTopicNames = this.ValidBagSelect.AvailableTopics.Properties.RowNames;
            
            % Filter topics with unsupported encoding types
            this.TopicSelectArray = {};
            for topicId = 1:numel(validTopicNames)
                topicName = validTopicNames{topicId};
                topicSelect = select(this.ValidBagSelect, 'Topic',...
                    topicName);
                
                msg = readMessages(topicSelect, 1, "DataFormat", "struct");
                msg = msg{1};
                
                messageType = topicSelect.AvailableTopics.MessageType;
                if messageType == "sensor_msgs/Image"
                    encoding = string(msg.Encoding);
                    allEncodings = string(ros.msg.sensor_msgs.internal.ImageEncoding.AllEncodings);
                    
                    if any(allEncodings == encoding)
                        this.TopicSelectArray{end+1} = topicSelect;
                    end
                elseif messageType == "sensor_msgs/CompressedImage"
                    compressedFormats = ["rgb8", "rgba8", "bgr8", "bgra8", "mono8"];
                    encoding = string(msg.Format);
                    if contains(encoding, compressedFormats)
                        this.TopicSelectArray{end+1} = topicSelect;
                    end
                else
                    this.TopicSelectArray{end+1} = topicSelect;
                end
            end
            
            
            numSignals = numel(this.TopicSelectArray);
            
            this.SignalName = strings(1, numSignals);
            this.SignalType = vision.labeler.loading.SignalType.empty(0, numSignals);
            this.Timestamp = cell(1, numSignals);
            
            
            for idx = 1:numSignals
                % Load time stamps
                topicSelect = this.TopicSelectArray{idx};
                validTopicName = topicSelect.AvailableTopics.Properties.RowNames{1};

                timeStamps = topicSelect.MessageList.Time;
                durationTime = seconds(timeStamps - timeStamps(1));
                this.Timestamp{idx} = durationTime;
                
                %Load signal names
                strTokens = strsplit(validTopicName, '/');
                signalName = strTokens{2};
                if numel(strTokens) > 2
                    for tokenId = 3:numel(strTokens)
                        signalName = signalName + "_" + strTokens{tokenId}; %#ok<AGROW>
                    end
                end
                this.SignalName(idx) = makeValidName(this, signalName, "rosbag_");
                
                %Add signal types
                signalType = this.ValidSignalTypes(this.ValidMessageTypes...
                    == string(topicSelect.AvailableTopics.MessageType));
                this.SignalType(idx) = signalType;
                
                this.TopicSelectArray{idx} = topicSelect;   
            end
            
            this.SourceName = sourceName;
            this.SourceParams = sourceParams;
        end
        
        function frame = readFrame(this, signalName, index)
            
            vision.internal.requiresROSToolbox(mfilename);
            
            signalId = find(this.SignalName == signalName, 1);

            msg = readMessages(this.TopicSelectArray{signalId}, index, "DataFormat", "struct");
            msg = msg{1};
            
            signalType = this.SignalType(signalId);
            
            if signalType == vision.labeler.loading.SignalType.Image
                frame = rosReadImage(msg);
            elseif signalType == vision.labeler.loading.SignalType.PointCloud
                xyz = rosReadXYZ(msg);
                frame = pointCloud(xyz);
            end
        end
    end
    
    methods(Access = private)
        
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
            noteTxtPadding = 75;
            noteTxtWidth = 350;
            noteTxtHeight = 75;
            
            yPosition = 50;
            noteYPosition = 10;
            
            filenameBoxX = fileNameTxtX + fileNameWidth + padding;
            fileNameButtonX = filenameBoxX + fileNameBoxWidth + padding;
            noteTxtX = fileNameButtonX + fileNameButtonWidth + noteTxtPadding;
            
            this.FileNameTxtPos     = [fileNameTxtX (yPosition - (widgetHeight/4)) fileNameWidth widgetHeight];
            this.FileNameBoxPos     = [filenameBoxX yPosition fileNameBoxWidth widgetHeight];
            this.FileNameButtonPos  = [fileNameButtonX yPosition fileNameButtonWidth widgetHeight];
            this.NoteTextPos        = [noteTxtX noteYPosition noteTxtWidth noteTxtHeight];
        end 
        
        %------------------------------------------------------------------
        % UI Components
        %------------------------------------------------------------------ 
        function addUIComponents(this)
            
            addFileBrowseComponents(this);
            
            addNotesComponents(this);
            
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
        
        function addNotesComponents(this)
            
            this.NoteText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:RosbagSourceNote'),...
                'Position', this.NoteTextPos,...
                'Tag', 'noteText','FontSize',7);                
            
        end
        
        %------------------------------------------------------------------
        % Callbacks
        %------------------------------------------------------------------ 
        
        function browseButtonCallback(this, ~, ~)
            persistent cachedPath
            
            if isempty(cachedPath)
                cachedPath = pwd;
            end
            
            disableBrowseButton(this);
            enableBrowse = onCleanup(@()set(this.FileBrowseButton, 'Enable', 'on'));      
            
            filterspec = {'*.bag'};
            dialogTitle = vision.getMessage('vision:labeler:SelectRosbagSource');    

            [fileName, filePath, filterIndex] = uigetfile(filterspec,...
                dialogTitle, cachedPath);
            
            hFig = ancestor(this.Panel, 'figure');
            figure(hFig);  
            
            userCanceled = (filterIndex == 0);
            
            if userCanceled
                return;
            end
            
            fullFileName = fullfile(filePath, fileName);
            
            this.FileNameBox.Value = fullFileName;
          
            cachedPath = filePath;
        end    
               
    end
    
    methods (Hidden)
        function isPending = checkPendingChanges(this)
            %checkPendingChanges Checks for pending changes in load panel
            %   checkPendingChanges returns a boolean flag isPending, if
            %   there are any pending changes in the load panel.
            
            isPending = ~isempty(this.FileNameBox) && ~isempty(this.FileNameBox.Value);
           
        end
        
        function disableBrowseButton(this)
            this.FileBrowseButton.Enable = 'off';
        end            
    end
        
end