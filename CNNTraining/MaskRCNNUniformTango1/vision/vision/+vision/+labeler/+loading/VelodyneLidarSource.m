%VelodyneLidarSource Loads the velodyne lidar signal to GroundTruth Labeler
%   VelodyneLidarSource is a MultiSignalSource object used to load signals
%   from Velodyne (TM) pcap files into the Ground Truth Labeler App.
%
%   Example: Create a velodyne lidar source
%   ---------------------------------------
%
%   % Specify the source file information
%   sourceName = fullfile(toolboxdir('vision'), 'visiondata',...
%       'lidarData_ConstructionRoad.pcap');
%
%   sourceParams = struct();
%   sourceParams.DeviceModel = 'HDL32E';
%   sourceParams.CalibrationFile = fullfile(matlabroot, 'toolbox', 'shared', ...
%         'pointclouds', 'utilities', 'velodyneFileReaderConfiguration', ...
%         'HDL32E.xml');
%
%   % Create and load velodyne lidar source
%   velodyneSource = vision.labeler.loading.VelodyneLidarSource();
%   velodyneSource.loadSource(sourceName, sourceParams);
%
%   % Read the fifth frame from the source
%   signalName = velodyneSource.SignalName;
%   pc = velodyneSource.readFrame(signalName, 1);
%   
%   figure, pcshow(pc)
%
%   See also vision.labeler.loading.MultiSignalSource

%   Copyright 2019-2023 The MathWorks, Inc.

classdef VelodyneLidarSource < vision.labeler.loading.MultiSignalSource
    
    properties
        Name = string(vision.getMessage('vision:labeler:VelodyneLidarDisplayName'))
        
        Description = string(vision.getMessage('vision:labeler:VelodyneLidarDes'))
    end

    %File reader properties
    properties (Access = private)
        FileReader
    end
    
    % Panel properties
    properties (Access = private)
        Panel
        
        FileNameText
        FileNameBox
        FileBrowseButton
        
        DeviceModelText
        DeviceModelDropDown
        
        CalibrationFileText
        CalibrationFileBox
        CalibrationFileBrowse
        
        TimeStampsText
        TimeStampsDropDown
        
        FileNameTxtPos
        FileNameBoxPos
        FileBrowseButtonPos
        DeviceModelTxtPos
        DeviceModelDropDownPos
        CalibrationFileTextPos
        CalibrationFileBoxPos
        CalibrationFileBrowsePos
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
            
            sourceParams = struct();
            sourceName = string(this.FileNameBox.Value);
            sourceParams.DeviceModel = this.DeviceModelDropDown.Value;
            sourceParams.CalibrationFile = this.CalibrationFileBox.Value;
                
        end
        
        %------------------------------------------------------------------
        % Load source
        %------------------------------------------------------------------          
        function loadSource(this, sourceName, sourceParams)
            
            % Load file
            deviceModel = sourceParams.DeviceModel;
            calibrationFile = sourceParams.CalibrationFile;
            
            this.FileReader = velodyneFileReader(sourceName, deviceModel, ...
                'CalibrationFile', calibrationFile);
            
            % Populate timestamps
            
            if isempty(this.Timestamp)
                if isfield(sourceParams, 'Timestamps')
                    setTimestamps(this, sourceParams.Timestamps);
                else
                    this.Timestamp = {(this.FileReader.Timestamps - this.FileReader.Timestamps(1))'};
                end
            else
                if ~iscell(this.Timestamp)
                    this.Timestamp = {this.Timestamp};
                end
            end

            % Populate signal names and types
            [~, fileName, ~] = fileparts(sourceName);
            
            this.SignalName = makeValidName(this, fileName, "velodyneLidar_");
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
                frame =  readFrame(this.FileReader, index);
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
            fileNameWidth = 100;
            fileNameBoxWidth = 300;
            fileNameButtonWidth = 70;
            
            deviceModelPadding = 100;
            deviceModelTxtWidth = 100;
            deviceModelDropwDownWidth = 100;
            
            calibrationFileNameWidth = 100;
            calibrationFileBoxWidth = 300;
            calibrationFileButtonWidth = 70;
            
            timeStampPadding = 100;
            timeStampTxtWidth = 100;
            timeStampDropwDownWidth = 150;  
            
            firstRowY = 70;
            filenameBoxX = fileNameTxtX + fileNameWidth + padding;
            fileNameButtonX = filenameBoxX + fileNameBoxWidth + padding;
            
            deviceModelTxtX = fileNameButtonX + fileNameButtonWidth + deviceModelPadding;
            deviceModelDropDownX = deviceModelTxtX + deviceModelTxtWidth + padding; 
            
            secondRowY = 20;
            
            calibrationFileBoxX = fileNameTxtX + calibrationFileNameWidth + padding;
            calibrationFileButtonX = calibrationFileBoxX + calibrationFileBoxWidth + padding;
            timeStampTxtX = calibrationFileButtonX + calibrationFileButtonWidth + timeStampPadding;
            timeStampDropDownX = timeStampTxtX + timeStampTxtWidth + padding;            
            
            this.FileNameTxtPos            = [fileNameTxtX (firstRowY - (widgetHeight/4)) fileNameWidth widgetHeight];
            this.FileNameBoxPos            = [filenameBoxX firstRowY fileNameBoxWidth widgetHeight];
            this.FileBrowseButtonPos       = [fileNameButtonX firstRowY fileNameButtonWidth widgetHeight];
            
            this.DeviceModelTxtPos         = [deviceModelTxtX firstRowY deviceModelTxtWidth widgetHeight];
            this.DeviceModelDropDownPos    = [deviceModelDropDownX firstRowY deviceModelDropwDownWidth widgetHeight];
            
            this.CalibrationFileTextPos    = [fileNameTxtX (secondRowY - (widgetHeight/4)) calibrationFileNameWidth widgetHeight];
            this.CalibrationFileBoxPos     = [calibrationFileBoxX secondRowY calibrationFileBoxWidth widgetHeight];
            this.CalibrationFileBrowsePos  = [calibrationFileButtonX secondRowY calibrationFileButtonWidth widgetHeight];
        
            this.TimeStampsTxtPos          = [timeStampTxtX secondRowY timeStampTxtWidth widgetHeight];
            this.TimeStampsDropDownPos     = [timeStampDropDownX secondRowY timeStampDropwDownWidth widgetHeight];
        end    
        
        %------------------------------------------------------------------
        % UI Components
        %------------------------------------------------------------------ 
        function addUIComponents(this)
            
            addFileBrowseComponents(this);
            
            addConfigComponents(this);
            
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
                'Position', this.FileBrowseButtonPos,...
                'ButtonPushedFcn', @this.fileBrowseButtonCallback,...
                'Tag', 'browseBtn');
            
        end
        
        function addConfigComponents(this)
            
            validDeviceModels = string(velodyneFileReader.validDeviceModels);
            
            calibFileName = fullfile(matlabroot, 'toolbox', 'shared',...
                'pointclouds', 'utilities',...
                'velodyneFileReaderConfiguration', 'VLP16.xml');            
            
            this.DeviceModelText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:DeviceModel'),...
                'Position', this.DeviceModelTxtPos,...
                'Tag', 'deviceModelText');
            
            this.DeviceModelDropDown = uidropdown('Parent', this.Panel,...
                'Items', validDeviceModels,...
                'Position', this.DeviceModelDropDownPos,...
                'ValueChangedFcn', @this.deviceModelDropDownCallback,...
                'Tag', 'deviceModelList');
            
            this.CalibrationFileText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:CalibrationFile'),...
                'Position', this.CalibrationFileTextPos,...
                'Tag', 'calibFileText');
            
            this.CalibrationFileBox = uieditfield('Parent', this.Panel,...
                'Value', calibFileName,...
                'Position', this.CalibrationFileBoxPos,...
                'Tag', 'calibFileEditBox');
            
            this.CalibrationFileBrowse = uibutton('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:Browse'),...
                'Position', this.CalibrationFileBrowsePos,...
                'ButtonPushedFcn', @this.calibBrowseButtonCallback,...
                'Tag', 'calibFileBrowseBtn');
            
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
        function fileBrowseButtonCallback(this, ~, ~)
            
            persistent cachedPath
            
            if isempty(cachedPath)
                cachedPath = pwd;
            end
            
            disableBrowseButton(this);
            enableBrowse = onCleanup(@()set(this.FileBrowseButton, 'Enable', 'on'));            
            
            filterspec = {'*.pcap'};
            dialogTitle = vision.getMessage('vision:labeler:LoadVelodyneFileDialogTitle');    

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
        
        function calibBrowseButtonCallback(this, ~, ~)
            persistent cachedPath
            
            if isempty(cachedPath)
                cachedPath = pwd;
            end
            
            this.CalibrationFileBrowse.Enable = 'off';
            enableBrowse = onCleanup(@()set(this.CalibrationFileBrowse, 'Enable', 'on'));     
            
            filterspec = {'*.xml'};  
            dialogTitle = vision.getMessage('vision:labeler:LoadVelodyneCalibFileDialogTitle');    
            
            [fileName, filePath, filterIndex] = uigetfile(filterspec,...
                dialogTitle, cachedPath);
            
            hFig = ancestor(this.Panel, 'figure');
            figure(hFig);
            
            userCanceled = (filterIndex == 0);
            
            if userCanceled
                return;
            end    
            
            fullFileName = fullfile(filePath, fileName); 
            
            this.CalibrationFileBox.Value = fullFileName;
           
            cachedPath = filePath;
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
        
        function deviceModelDropDownCallback(this, ~, ~)
            
            deviceModel = this.DeviceModelDropDown.Value;
          
            fileName = strcat(deviceModel, '.xml');
            calibFileName = fullfile(matlabroot, 'toolbox', 'shared',...
                'pointclouds', 'utilities',...
                'velodyneFileReaderConfiguration', fileName);
            
            this.CalibrationFileBox.Value = calibFileName;
                     
        end           
    end
    
    methods (Hidden)
        function sourceParams = fixSourceParams(this, alternatePaths)
            
            origPath = alternatePaths(1);
            currentPath = alternatePaths(2);
            
            sourceParams = this.SourceParams;
            
            calibFile = sourceParams.CalibrationFile;
            if ~exist(calibFile, 'file')
                try
                    sourceParams.CalibrationFile = vision.internal.uitools.tryToAdjustPath(char(calibFile),...
                        char(currentPath), char(origPath));
                catch
                    fileName = strcat(sourceParams.DeviceModel, '.xml');
                    sourceParams.CalibrationFile = fullfile(matlabroot, 'toolbox', 'shared',...
                        'pointclouds', 'utilities',...
                        'velodyneFileReaderConfiguration', fileName);
                end
            end
        end
        
        function isPending = checkPendingChanges(this)
            %checkPendingChanges Checks for pending changes in load panel
            %   checkPendingChanges returns a boolean flag isPending, if
            %   there are any pending changes in the load panel.
            isPending = ~isempty(this.FileNameBox.Value);                
            
        end
        
        function disableBrowseButton(this)
            this.FileBrowseButton.Enable = 'off';
        end                 
    end
end
