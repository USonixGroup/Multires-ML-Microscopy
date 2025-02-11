% The class defines and creates the Board thumbnails Browser for Camera
% Calibrator and Stereo Camera Calibrator App.

% Copyright 2021-2024 The MathWorks, Inc.
classdef  BoardThumbnailBrowser < handle
    
    %----------------------------------------------------------------------
    % Thumbnail properties.
    %----------------------------------------------------------------------
    properties (GetAccess = ?uitest.factory.Tester)
        BrowserObj
    end

    properties (Access = private)
        % Callback Handles
        SelectionListener = [];
        RemoveListener = [];
        WinScrollListener = [];
        KeyPressListener = [];

        ThumbnailSize
        BrowserFigure

        % Image Capture Board Names
        BoardName

        % Capture the theme
        Theme
    end
    
     %----------------------------------------------------------------------
    % Properties to store callback function handles for thumbnail events.
    %----------------------------------------------------------------------
    properties
        % SelectionCallback Callback for selecting an image with the mouse.
        % Typically, apps will use this callback to draw the image in the
        % apps main display.
        SelectionCallback = [];
        
        % SelectionRemoveCallback Callback for deleting the selected image.
        SelectionRemoveCallback = [];
        
        % SelectionRemoveAndRecalibrateCallback Callback for deleting the
        % selected image after calibration. Session will be recalibrated
        % after deletion.
        SelectionRemoveAndRecalibrateCallback = [];
    end
    
    %----------------------------------------------------------------------
    % Public methods
    %----------------------------------------------------------------------
    methods
        %------------------------------------------------------------------
        function this = BoardThumbnailBrowser(hParent)
            this.BrowserFigure = hParent.Fig;
            matlab.graphics.internal.themes.figureUseDesktopTheme(this.BrowserFigure);
            this.Theme = this.BrowserFigure.Theme;
            this.BrowserObj = images.internal.app.browser.Browser(...
                this.BrowserFigure, ...
                [1, 1, this.BrowserFigure.Position(3:4)]);
           
            this.BrowserObj.LabelVisible = true;
            this.BrowserObj.Layout = 'column';
            setBrowserObjColors(this);
           
            this.BrowserFigure.ThemeChangedFcn = @(src,evt)this.reactToThemeChange(src,evt);
            this.BrowserFigure.SizeChangedFcn = @(src,evt) resize(this.BrowserObj, [1, 1, src.Position(3:4)]);
            
            this.WinScrollListener = addlistener(this.BrowserFigure, 'WindowScrollWheel', @(src,evt) scroll(this.BrowserObj, evt.VerticalScrollCount));
            this.KeyPressListener = addlistener(this.BrowserFigure, 'KeyPress', @(src,evt) this.keyPressCallback(evt));
            
            % To prevent focus steal
            this.BrowserFigure.KeyPressFcn = @(varargin)[];

            % Workaround for g2583131
            resize(this.BrowserObj, [1, 1, this.BrowserFigure.Position(3:4)]);
        end
        
        %------------------------------------------------------------------
        function setBrowserContent(this, boardSet, selectThumbnail)

            lastSelectedThumbnail = 1;
            if nargin < 3
                selectThumbnail = true;
                if ~isempty(this.BrowserObj.Selected)
                    lastSelectedThumbnail = this.BrowserObj.Selected;

                end
            end
            % Add Images in Thumbnail data Browser
            clearBrowser(this);
            this.setThumbnailSize(boardSet.FullPathNames, boardSet.IsStereo);
            
            if boardSet.IsStereo
                addStereoCameraImages(this, boardSet);
            else
                addMonoCameraImages(this, boardSet);
            end
            
            % Select first thumbnail entry if no items was selected 
            if selectThumbnail && isempty(this.BrowserObj.Selected)
                this.BrowserObj.select(lastSelectedThumbnail);
            end
        end
        
        %------------------------------------------------------------------
        function addStereoCameraImages(this, boardSet)
            this.BrowserObj.LabelLocation = 'bottom';

            concatData = num2cell([boardSet.FullPathNames; boardSet.PatternLabels],1);
            add(this.BrowserObj,  concatData);
            this.BrowserObj.ReadFcn = @this.readFcnStereo;

            addBadges(this, boardSet.PartialPatterns);
        end
        
        %------------------------------------------------------------------
        function addMonoCameraImages(this, boardSet)
            this.BrowserObj.LabelLocation = 'right';

            add(this.BrowserObj,  boardSet.FullPathNames);
            this.BrowserObj.ReadFcn = @this.readFcnMono;

            addBadges(this, boardSet.PartialPatterns);
        end
        
        %------------------------------------------------------------------
        function addBadges(this, checkerboardPattern)
            for imagesId = 1:size(checkerboardPattern,1)
                if checkerboardPattern(imagesId)
                    % Add Badges to indicate partial-checkerbaord
                    this.BrowserObj.setBadge(imagesId, images.internal.app.browser.data.Badge.PartialCheckerboard);
                else
                    % Add Badges to indicate full-checkerboard
                    this.BrowserObj.setBadge(imagesId, images.internal.app.browser.data.Badge.FullCheckerboard);
                end
            end
        end

        %------------------------------------------------------------------
        function [thumbnail, label, badge, userData] = readFcnMono(this, source)
            
            if ~ischar(source)
                % Retrieve Data while App is in live Image Capture mode
                [thumbnail, label, badge, userData] = imageCaptureReadFcn(this, source);
                return;
            end

            try
                fullImage = imread(source);
                fullImage = correctDimensionOfImage(fullImage);
            catch ALL %#ok<NASGU>
                fullImage = imread(fullfile(matlabroot,'toolbox','images','imuitools',...
                    '+images','+internal','+app','+browser','+icons','BrokenPlaceholder_100.png'));
            end
            
            % Resize thumbnail.
            % The Camera Calibrator App thumbnail has fixed height.
            % The width varies according to image width-height ratio.
            thumbnail = imresize(fullImage, this.ThumbnailSize);
            
            [~, name, extn] = fileparts(source);
            imagesId = identifyIndexMono(this.BrowserObj.Sources, source);
            label(1) = string([num2str(imagesId),':']);
            label(2) = [name, extn];

            badge = images.internal.app.browser.data.Badge.Empty;
            userData = struct();
        end
        
        %------------------------------------------------------------------
        function [thumbnail, label, badge, userData] = readFcnStereo(this, source)
            try
                fullImage1 = imread(source{1});
                fullImage2 = imread(source{2});

                % Correct thumbnail image content
                fullImage1 = correctDimensionOfImage(fullImage1);
                fullImage2 = correctDimensionOfImage(fullImage2);
                
            catch ALL %#ok<NASGU>
                fullImage1 = imread(fullfile(matlabroot,'toolbox','images','imuitools',...
                    '+images','+internal','+app','+browser','+icons','BrokenPlaceholder_100.png'));
                fullImage2 = fullImage1;
            end

            % Resize thumbnail.
            % The Stereo Camera Calibrator App thumbnail has fixed height.
            % The width varies according to image width-height ratio.
            thumbnail = this.resizeToThumbnailStereo(fullImage1, fullImage2);

            index = identifyIndex(this.BrowserObj.Sources, source);
            label = num2str(index)+": "+string(source{3});
            badge = images.internal.app.browser.data.Badge.Empty;
            userData = struct();
        end
       
        %------------------------------------------------------------------
        % Initialize Live Camera Captured Thumbnail.
        %------------------------------------------------------------------
        function setImageCaptureThumbnail(this, boardImage, boardName)
            if isempty(this.BrowserObj.ThumbnailVisible)
                this.BrowserObj.LabelLocation = 'right';
                this.BrowserObj.ReadFcn = @this.imageCaptureReadFcn;
                this.BoardName = convertCharsToStrings(boardName{1});
            end
            this.BoardName = convertCharsToStrings(boardName{1});
            add(this.BrowserObj,  {boardImage}, length(this.BrowserObj.ThumbnailVisible)+1);
            selectItem(this, this.getNumImages());
        end

        %------------------------------------------------------------------
        function [im, label, badge, userData] = imageCaptureReadFcn(this, source)
            im =  source(:,:,:,end);
            label = this.BoardName;
            badge = images.internal.app.browser.data.Badge.Empty;
            userData = struct();
        end

        %------------------------------------------------------------------
        function clearBrowser(this)
            clear(this.BrowserObj);
        end

        %------------------------------------------------------------------
        function deleteBrowser(this)
            % Reset Browser Resize callback
            if isvalid(this.BrowserFigure)
                this.BrowserFigure.SizeChangedFcn = [];
            end

            % Delete Browser and associated objects
            delete(this.BrowserObj);

            % Delete Listener
            delete(this.SelectionListener);
            delete(this.RemoveListener);
            delete(this.WinScrollListener);
            delete(this.KeyPressListener);
        end
        
        %------------------------------------------------------------------
        function itemNo = selectedItem(this)
            itemNo = this.BrowserObj.Selected();
        end
        
        %------------------------------------------------------------------
        function numImage = getNumImages(this)
            numImage = this.BrowserObj.NumImages();
        end
        
        %------------------------------------------------------------------
        function selectItem(this, entryNo)
            this.BrowserObj.select(entryNo);
        end
        
        %------------------------------------------------------------------
        function setSelectedIndices(this, entryNo)
            newSelection = [this.BrowserObj.Selected; entryNo'];
            this.selectItem(newSelection);
        end
        
        %------------------------------------------------------------------
        function removeSelected(this, isStereo)
            lastSelected = min(this.BrowserObj.Selected);
            this.BrowserObj.removeSelected();
            
            if isStereo
                % The label can have varying length and this lead to a
                % concatenation error. So, create a cell array and convert
                % it to char matrix to avoid concatenation error.
                labels = [];
                for imgIdx = lastSelected:this.getNumImages()
                    [~, name1, extn1] = fileparts(this.BrowserObj.Sources{imgIdx,1}{1});
                    [~, name2, extn2] = fileparts(this.BrowserObj.Sources{imgIdx,1}{2});
                    label = [num2str(imgIdx),': ', name1, extn1, ' & ', name2, extn2];
                    labels{imgIdx - lastSelected + 1} = label;
                end
                allLabel = char(labels);
            else
                allLabel = [];
                for imgIdx = lastSelected:this.getNumImages()
                    [~, name, extn] = fileparts(this.BrowserObj.Sources{imgIdx});
                    label(1) = string([num2str(imgIdx),':']);
                    label(2) = [name, extn];
                    allLabel = [allLabel, label];
                end
            end
            
            if ~isempty(allLabel)
                this.BrowserObj.setMultilineLabel(lastSelected:this.getNumImages(), allLabel);
            end
        end
        
        %------------------------------------------------------------------
        function disableContextMenu(this)
            delete(this.BrowserObj.ContextMenu.Children);
        end
        
        %------------------------------------------------------------------
        function keyPressCallback(this, evt)
            images.internal.app.browser.helper.keyPressCallback(this.BrowserObj, evt);
            this.SelectionRemoveCallback([], evt);
        end
        
        %------------------------------------------------------------------
        function removeBrowserCallbacks(this)
            % Delete the Listener callbacks
            delete(this.WinScrollListener);
            delete(this.KeyPressListener);
            
            % Clear the deleted handle
            this.WinScrollListener = [];
            this.KeyPressListener = [];
        end
        
        %------------------------------------------------------------------
        function addSelectionListener(this)
            % API hook ups
            this.SelectionListener = addlistener(this.BrowserObj,'SelectionChanged',this.SelectionCallback);
            this.RemoveListener = addlistener(this.BrowserObj,'OpenSelection',this.SelectionRemoveCallback);
        end
        
        %------------------------------------------------------------------
        function updateContextMenuMessage(this, message)
            % Update 
            installRemoveContextMenu(this);
            this.BrowserObj.ContextMenu.Children.Text = message;
        end
        
        %------------------------------------------------------------------
        function installRemoveContextMenu(this)
            if isempty(this.BrowserObj.ContextMenu.Children)
                uimenu(this.BrowserObj.ContextMenu, 'Label',...
                    vision.getMessage('vision:uitools:Remove'),...
                    'Callback', @(src,evt)this.SelectionRemoveAndRecalibrateCallback(),...
                    'Tag', 'ContextMenuRemove');
            end
        end
        
        %------------------------------------------------------------------
        function removeSelectionListener(this)
            % Delete the Listener callbacks
            delete(this.SelectionListener);
            delete(this.RemoveListener);
           
            % Clear the deleted handle
            this.SelectionListener = [];
            this.RemoveListener = [];
        end

    end
   
    methods (Access = private)
        %------------------------------------------------------------------
        % Resize thumbnail
        function thumbnail = resizeToThumbnailStereo(this, fullImage1, fullImage2)
            imageSize =  this.ThumbnailSize;
            imageSize(2) = imageSize(2)/2 - 1;
            image1 = im2uint8(imresize(fullImage1, imageSize));
            image2 = im2uint8(imresize(fullImage2, imageSize));
            
            thumbnail = vision.internal.calibration.tool.fuseWithSeparator(image1, image2);
        end
        
        %------------------------------------------------------------------
        function setThumbnailSize(this, imageFullPath, isStereo)
            this.ThumbnailSize = [72 72];
            
            % Read at-least one image from directory to set the board
            % thumbnail width a/c with fixed height
            frameNo = 1;
            RefImage = [];
            while isempty(RefImage)
                try
                    RefImage = imread(imageFullPath{frameNo});
                catch
                    frameNo = frameNo + 1;
                end
            end
            imgSize = size(RefImage);
            
            if isStereo
                this.updateStereoBoardThumbnailSize(imgSize)
            else
                this.updateBoardThumbnailSize(imgSize)
            end
        end
        
        %------------------------------------------------------------------
        function updateBoardThumbnailSize(this, imgSize)
            this.ThumbnailSize(2) = round(this.ThumbnailSize(1)*imgSize(2)/imgSize(1));
        end
        
        %------------------------------------------------------------------
        function updateStereoBoardThumbnailSize(this, imgSize)
            this.ThumbnailSize(2) = round(this.ThumbnailSize(1)*imgSize(2)/imgSize(1))*2 + 2;
        end

        %------------------------------------------------------------------
        function setBrowserObjColors(this)
            if isvalid(this.BrowserObj) 
                % When a new session is loaded, only the browser figure is available.
                % Since browser obj is not created, do not update anything
                % in it.
                this.BrowserObj.BackgroundColor = matlab.graphics.internal.themes.getAttributeValue...
                    (this.Theme, '--mw-backgroundColor-primary');
                this.BrowserObj.SelectedColor = matlab.graphics.internal.themes.getAttributeValue...
                    (this.Theme, '--mw-graphics-colorOrder-1-secondary');
                this.BrowserObj.LastSelectedColor = matlab.graphics.internal.themes.getAttributeValue...
                    (this.Theme, '--mw-backgroundColor-selectedFocus');
                this.BrowserObj.LabelTextColor = matlab.graphics.internal.themes.getAttributeValue...
                    (this.Theme, '--mw-backgroundColor-gutter-currentExecutingLine');
            end

            if ~isempty(this.BrowserFigure.Children) && isscalar(numel(this.BrowserFigure.Children))
                %This scenario occurs when all the thumbnails are deleted
                %and only the data browser message is left
                grid = this.BrowserFigure.Children;
                theme = this.BrowserFigure.Theme;
                if isa(grid,'matlab.ui.container.GridLayout')
                   grid.BackgroundColor = matlab.graphics. ...
                    internal.themes.getAttributeValue(theme,...
                    '--mw-backgroundColor-primary');
                end
            end
        end

        %------------------------------------------------------------------
        function reactToThemeChange(this,src,evt)
            this.Theme = this.BrowserFigure.Theme;
            setBrowserObjColors(this);
        end
    end
end

%--------------------------------------------------------------------------
function image = correctDimensionOfImage(image)
    % Check the Image dimension and correct to valid Dimension
    if ndims(image)>3 || ( size(image,3)~=1 &&size(image,3)~=3)
        % pick first plane
        image = image(:,:,1);
        image = repmat(image, [1,1,3]);
        return;
    end
    
    if size(image, 3)==1
        image = repmat(image, [1,1,3]);
    end
end

%------------------------------------------------------------------
function idx = identifyIndex(sourceData, source)
    for idx = 1:length(sourceData)
        if strcmp(sourceData{idx,1}{3,1}, source{3,1})
            break;
        end
    end
end

%------------------------------------------------------------------
function idx = identifyIndexMono(sourceData, source)
    for idx = 1:length(sourceData)
        if strcmp(sourceData{idx}, source)
            break;
        end
    end
end

