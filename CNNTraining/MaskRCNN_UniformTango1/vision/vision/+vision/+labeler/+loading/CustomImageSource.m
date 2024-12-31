%CustomImageSource Loads the images from custom sources to the GroundTruth Labeler
%   CustomImageSource is a MultiSignalSource object that is used to load
%   signal consisting of images of custom format, to the Ground Truth
%   Labeler App. 
%
%   Example: Load a custom image source
%   -----------------------------------
%   % Specify image directory containing sequence of road images
%   imageDir = fullfile(toolboxdir('vision'),'visiondata','building');
%
%   % Use an image data store as a custom data source
%   imgDataStore = imageDatastore(imageDir);
%
%   % Write a reader function to read images from the data source. The
%   % first input argument to readerFcn, sourceName is not used. The
%   % 2nd input, currentTimeStamp is converted from a duration scalar
%   % to a 1-based index suitable for the data source.
%   readerFcn = @(~,idx)readimage(imgDataStore, seconds(idx));
%
%   % Create a custom image source
%   sourceName = imageDir;
%   sourceParams = struct();
%   sourceParams.FunctionHandle = readerFcn;
%   sourceParams.Timestamps = seconds(1:5);
%
%   customImgSource = vision.labeler.loading.CustomImageSource()
%   customImgSource.loadSource(sourceName, sourceParams);
%
%   % Read the 5th frame in the sequence
%   signalName = customImgSource.SignalName;
%   I = customImgSource.readFrame(signalName, 5);
%
%   figure, imshow(I)
%
%   See also vision.labeler.loading.MultiSignalSource

% Copyright 2019-2023 The MathWorks, Inc.

classdef CustomImageSource < vision.labeler.loading.MultiSignalSource
    
    properties
        Name = string(vision.getMessage('vision:labeler:CustomImageDisplayName'))
        
        Description = string(vision.getMessage('vision:labeler:CustomImageDes'))
    end
    
    properties (Access = private)
        FunctionHandle
    end
    
    % Panel Properties
    properties (Access = private)
        Panel
        ReaderFcnText
        ReaderFcnBox
        SourceNameText
        SourceNameBox        
        TimeStampsText
        TimeStampsButton
        
        ReaderFcnTextPos
        ReaderFcnBoxPos
        SourceNameTextPos
        SourceNameBoxPos  
        TimeStampsTxtPos
        TimeStampsButtonPos      
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
            sourceName = string(this.SourceNameBox.Value);
            sourceParams.FunctionHandle = string(this.ReaderFcnBox.Value);
        end
        
        function loadSource(this, sourceName, sourceParams)
            
            this.SourceName = sourceName;
            
            functionHandle = sourceParams.FunctionHandle;
            if ~isa(functionHandle, 'function_handle')
                
                absolutePathFileName = which(functionHandle);
                
                if(isempty(absolutePathFileName))
                    errorMessage = vision.getMessage('vision:labeler:pathNotFound', functionHandle);
                    error(errorMessage);
                    return;
                end
        
                [~,fileName,fileExt] = fileparts(absolutePathFileName);
                
                if ~(exist(fileName,'file') && strcmpi(fileExt,'.m'))
                    errorMessage = vision.getMessage('vision:labeler:CustomReaderNotValid', functionHandle);
                    error(errorMessage);                    
                end
                
                functionHandle = str2func(fileName);
            end
            
            this.FunctionHandle = functionHandle;
            
            if isfield(sourceParams, 'Timestamps')
                setTimestamps(this, sourceParams.Timestamps);
            end
            
            this.SourceParams = sourceParams;
            
            import vision.internal.labeler.validation.*;
            
            if ~isempty(this.Timestamp)
                validateCustomReaderFunction(this.FunctionHandle,this.SourceName,this.Timestamp{1});
            end
            
            this.SignalName = makeValidName(this, sourceName, "customImage_");
            this.SignalType = vision.labeler.loading.SignalType.Image;
        end
        
        function frame = readFrame(this, signalName, index)
            if strcmpi(signalName, this.SignalName)
                frame = this.FunctionHandle(this.SourceName,...
                    this.Timestamp{1}(index));
            else
                frame = [];
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
            
            readerFcnTxtX = 10;
            readerFcnTxtWidth = 175;
            readerFcnBoxWidth = 200;
            sourceNameTxtWidth = 175;
            sourceNameBoxWidth = 200;
            
            timeStampPadding = 100;
            timeStampTxtWidth = 90;
            timeStampBtnWidth = 250;            
            
            firstRowY = 70;
            secondRowY = 20;
            
            readerFcnBoxX = readerFcnTxtX + readerFcnTxtWidth + padding;
            sourceNameBoxX = readerFcnTxtX + sourceNameTxtWidth + padding;
            timestampTxtX = readerFcnBoxX + readerFcnBoxWidth + timeStampPadding;
            timestampBtnX = timestampTxtX + timeStampTxtWidth;
            
            this.ReaderFcnTextPos = [readerFcnTxtX (firstRowY - (widgetHeight/4)) readerFcnTxtWidth widgetHeight];
            this.ReaderFcnBoxPos = [readerFcnBoxX firstRowY readerFcnBoxWidth widgetHeight];
            this.SourceNameTextPos = [readerFcnTxtX (secondRowY - (widgetHeight/4)) sourceNameTxtWidth widgetHeight];
            this.SourceNameBoxPos = [sourceNameBoxX secondRowY sourceNameBoxWidth widgetHeight];
            this.TimeStampsTxtPos = [timestampTxtX firstRowY timeStampTxtWidth widgetHeight];
            this.TimeStampsButtonPos = [timestampBtnX firstRowY timeStampBtnWidth widgetHeight];
        end
        
        %------------------------------------------------------------------
        % UI Components
        %------------------------------------------------------------------ 
        function addUIComponents(this)
            
            addFileBrowseComponents(this);
            
            addTimeStampsComponents(this);
            
        end  
        
        function addFileBrowseComponents(this)
            
            this.ReaderFcnText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:CustomReaderFunction'),...
                'Position', this.ReaderFcnTextPos,...
                'Tag', 'readerFcnTxt');
            
            this.ReaderFcnBox = uieditfield('Parent', this.Panel,...
                'Value', '',...
                'Position', this.ReaderFcnBoxPos,...
                'Tag', 'readerFcnTxtBox');
            
            this.SourceNameText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:SourceName'),...
                'Position', this.SourceNameTextPos,...
                'Tag', 'sourceNameTxt');
            
            this.SourceNameBox = uieditfield('Parent', this.Panel,...
                'Value', '',...
                'Position', this.SourceNameBoxPos,...
                'Tag', 'sourceNameTxtBox');
                      
        end
        
        function addTimeStampsComponents(this)
            
            this.TimeStampsText = uilabel('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:Timestamps'),...
                'Position', this.TimeStampsTxtPos,...
                'Tag', 'timeStampTxt');
            
            this.TimeStampsButton = uibutton('Parent', this.Panel,...
                'Text', vision.getMessage('vision:labeler:ImportfromWorkspace'),...
                'Position', this.TimeStampsButtonPos,...
                'ButtonPushedFcn', @this.timeStampsBtnCallback,...
                'Tag', 'timeStampSourceSelectList');                
           
        end
        
        %------------------------------------------------------------------
        % Callbacks
        %------------------------------------------------------------------ 
        
        function timeStampsBtnCallback(this, ~, ~)
            variableTypes = {'duration'};
            variableDisp =  {'duration'};
            [timestamps,~,isCanceled] = vision.internal.uitools.getVariablesFromWS(variableTypes, variableDisp);
            
            if isCanceled
                return;
            end
            
            if isempty(timestamps)
                errorMessage = vision.getMessage('vision:labeler:ImportTimeStampsFirst');
                error(errorMessage);
            end
            
            vision.internal.labeler.validation.validateTimestamps(timestamps);
            
            setTimestamps(this, timestamps);
        end
             
    end
    
    methods (Hidden)
        function isPending = checkPendingChanges(this)
            %checkPendingChanges Checks for pending changes in load panel
            %   checkPendingChanges returns a boolean flag isPending, if
            %   there are any pending changes in the load panel.

            isPending = ~isempty(this.SourceNameBox.Value) ||...
                ~isempty(this.ReaderFcnBox.Value);
            
        end
    end        
end