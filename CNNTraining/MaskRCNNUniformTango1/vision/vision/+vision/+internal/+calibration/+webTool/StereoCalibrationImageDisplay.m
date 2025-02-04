classdef StereoCalibrationImageDisplay < vision.internal.calibration.webTool.CalibrationImageDisplay
    % StereoCalibrationImageDisplay Class that encapsulates the main image display
    %   in the Stereo Calibrator App
    %
    % StereoCalibrationImageDisplay methods:
    %   drawBoard - display the calibration image pair with annotations
    
    % Copyright 2014-2024 The MathWorks, Inc.
    
    properties(Access=private)
        IsImage1Valid = true;
        IsImage2Valid = true;
    end
    
    properties
        MessageState = false
        MessagePaneButton
        MessagePane
        CameraLabelText
        MissingImageText
    end
    
    methods
        function this = StereoCalibrationImageDisplay(options)
            this = this@vision.internal.calibration.webTool.CalibrationImageDisplay(options);
        end       
        %------------------------------------------------------------------
        function drawBoard(this, board, boardIndex, stereoParams)
            % drawBoard Display the calibration image with annotations.
            %   drawBoard(obj, board, boardIdx, stereoParams) renders the
            %   calibration image, detected points, and board axes labels. If
            %   cameraParams input is non-empty, the method also plots the
            %   reprojected points.
            %
            %   Inputs:
            %   ------
            %   obj          - A StereoCalibrationImageDisplay object
            %
            %   board        - A struct containing the following fields:
            %                  boardSize      - [rows,cols]
            %                  fileName       - 2-element cell array of image file names
            %                  label          - title of the image axes
            %                  detectedPoints - M-by-2-by-1-by-2 array of x-y coordinates
            %
            %   boardIdx     - Index of this image/board
            %
            %   stereoParams - A stereoParameters object
            if ~ishandle(this.Figure)
                return;
            end
            
            drawOriginalImages(this, board, boardIndex, stereoParams);
        end
        
        %------------------------------------------------------------------
        function drawOriginalImages(this, board, boardIndex, stereoParams)
            [I1, I2] = readImages(this, board.fileName);
            
            [I, offset] = ...
                vision.internal.calibration.tool.fuseWithSeparator(I1, I2);
            
            makeHandleVisible(this);
            drawImage(this, I);
            
            [p1, p2] = getDetectedPoints(this, board, offset);
            
            center1 = round([size(I1, 1), size(I1, 2)] / 2);
            center2 = [center1(1), center1(2) + offset];
            showMissingImageWarnings(this, center1, center2, board.fileName);
            
            if this.IsImage1Valid || this.IsImage2Valid
                if isa(board.detector, 'vision.calibration.stereo.CheckerboardDetector')...
                        || isa(board.detector, 'vision.internal.calibration.stereo.CheckerboardDetector')
                    plotOrigin = false;
                else
                    plotOrigin = true;
                end
                plotDetectedPoints(this, board.detector, p1, p2, plotOrigin);
                
                if ~isempty(stereoParams) && (boardIndex(1) <= stereoParams.NumPatterns)
                    plotReprojectedPoints(this, stereoParams, boardIndex, offset);
                end
                
                drawCoordinateAxes(this, board.detector, p1, p2);
                
                showLegend(this);
            end
            setTitle(this, board.label);
            
            drawCameraLabels(this, I1, I);
            
            holdOff(this);
            set(this.Figure,'HandleVisibility','callback');
        end
        
        %------------------------------------------------------------------
        function drawRectifiedImages(this, board, stereoParams)
            [I1, I2] = readImages(this, board.fileName);
            try
                prevWarning = warning('OFF', 'vision:calibrate:switchValidViewToFullView');
                cl = onCleanup(@()warning(prevWarning));
                [I1, I2] = rectifyStereoImages(I1, I2, stereoParams);
                cl.delete();
            catch e
                error(message('vision:caltool:unableToRectifyImages'));
            end
            
            I = vision.internal.calibration.tool.fuseWithSeparator(I1, I2);
            drawImage(this, I);
            plotEpipolarLines(this, I);
            
            drawCameraLabels(this, I1, I);
            
            setTitle(this, board.label);
            holdOff(this);
            set(this.Figure,'HandleVisibility','callback');
        end
        
        function addMessagePane(this, msg)
            % iconPath = fullfile(toolboxdir('vision'), 'vision', '+vision', '+internal','+ocr','+tool');
            
            figPosition = this.Figure.Position;
            iconPos = [5 figPosition(4)-21 17 17];
            this.MessagePaneButton = uiimage(this.Figure, 'Position', iconPos);
            matlab.ui.control.internal.specifyIconID(this.MessagePaneButton, 'showUI', 16);
            this.MessagePaneButton.ImageClickedFcn = @this.updateMessageState;
            
            this.Figure.AutoResizeChildren = 'off';
            this.Figure.SizeChangedFcn = @this.updateMessagePanSize;
            
            
            panPosition = [22 figPosition(4)-24 figPosition(3)-26 22];
            paneColor = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-notificationBanner');
            this.MessagePane = uipanel('Parent', this.Figure,...
                'Tag','MessagePanel',...
                'Visible','off',...
                'BackgroundColor',paneColor,...
                'Position',panPosition);
            this.MessagePane.Scrollable = 'on';
            
            [~] = uilabel(this.MessagePane,...
                'Text',msg,...
                'HorizontalAlignment', 'left', ...
                'Position',[1 1 panPosition(3)-20 panPosition(4)],...
                'Tag', 'intro text');
            
            
            if ~this.MessageState
                this.MessagePane.Visible = 'on';
                matlab.ui.control.internal.specifyIconID(this.MessagePaneButton, 'showUI', 16);
            else
                this.MessagePane.Visible = 'off';
                matlab.ui.control.internal.specifyIconID(this.MessagePaneButton, 'hideUI', 16);
            end
        end
        
        function updateMessageState(this, ~, ~)
            if this.MessageState
                this.MessageState = false;
                this.MessagePane.Visible = 'on';
                matlab.ui.control.internal.specifyIconID(this.MessagePaneButton, 'showUI', 16);
            else
                this.MessageState = true;
                this.MessagePane.Visible = 'off';
                matlab.ui.control.internal.specifyIconID(this.MessagePaneButton, 'hideUI', 16);

            end
        end
        
        function updateMessagePanSize(this,~, ~)
            if ~isempty(this.MessagePane) && isvalid(this.MessagePane)
                figPosition = this.Figure.Position;
                iconPos = [5 figPosition(4)-21 17 17];
                
                this.MessagePaneButton.Position = iconPos;
                
                panPosition = [22 figPosition(4)-24 figPosition(3)-26 22];
                this.MessagePane.Position = max(1, panPosition);
                
                this.MessagePane.Children.Position(3) = max(1,this.Figure.Position(3)-46);
            end
        end
        
        %------------------------------------------------------------------
        function showMessagePane(this)
            
            if ~isempty(this.MessagePane) && isvalid(this.MessagePane)
                this.MessagePaneButton.Visible = 'on';
                this.MessagePane.Visible = 'on';
            end
            
        end
        
        %------------------------------------------------------------------
        function hideMessagePane(this)
            
            if ~isempty(this.MessagePane) && isvalid(this.MessagePane)
                this.MessagePaneButton.Visible = 'off';
                this.MessagePane.Visible = 'off';
            end
        end
    end
    
    methods(Access=private)
        
        %------------------------------------------------------------------
        function [I1, I2] = readImages(this, fileNames)
            % Read image 1
            try
                I1 = imread(fileNames{1});
                this.IsImage1Valid = true;
            catch
                I1 = [];
                this.IsImage1Valid = false;
            end
            
            % Read image 2
            try
                I2 = imread(fileNames{2});
                this.IsImage2Valid = true;
            catch
                I2 = [];
                this.IsImage2Valid = false;
            end
            
            % Deal with missing images
            if isempty(I1) && isempty(I2)
                emptyImageSize = [480, 640];
                I1 = zeros(emptyImageSize);
                I2 = zeros(emptyImageSize);
            elseif isempty(I1)
                I1 = zeros(size(I2), 'like', I2);
            elseif isempty(I2)
                I2 = zeros(size(I1), 'like', I1);
            end
            
            % Handle the case when one image is grayscale and the other RGB
            if size(I1, 3) ~= size(I2, 3)
                if ismatrix(I1)
                    I1 = repmat(I1, [1,1,3]);
                else
                    I2 = repmat(I2, [1,1,3]);
                end
            end
        end
        
        %------------------------------------------------------------------
        function [p1, p2] = getDetectedPoints(this, board, offset)
            if this.IsImage1Valid
                p1 = board.detectedPoints(:,:,:, 1);
            else
                p1 = zeros(0, 2);
            end
            
            if this.IsImage2Valid
                p2 = board.detectedPoints(:,:,:, 2);
                p2(:, 1) = p2(:, 1) + offset;
            else
                p2 = zeros(0, 2);
            end
        end
        
        %------------------------------------------------------------------
        function showMissingImageWarnings(this, center1, center2, fileNames)
            hAxes = getImageAxes(this);
            color = this.setTextColor();
            if ~this.IsImage1Valid
                this.MissingImageText = text(center1(2), center1(1), ['Cannot read ', fileNames{1}], ...
                    'Color', color, 'Parent', hAxes, 'HorizontalAlignment', 'center');
            end
            
            if ~this.IsImage2Valid
                this.MissingImageText = text(center2(2), center2(1), ['Cannot read ', fileNames{2}], ...
                    'Color', color, 'Parent', hAxes, 'HorizontalAlignment', 'center');
            end
        end
        
        %------------------------------------------------------------------
        function drawCameraLabels(this, I1, I)
            loc1 = [size(I1, 2)-10, size(I, 1) - 20];
            drawCameraLabel(this, loc1, 'Camera 1');
            
            loc2 = [size(I, 2)-10, size(I, 1) - 20];
            drawCameraLabel(this, loc2, 'Camera 2');
        end
        
        %------------------------------------------------------------------
        function drawCameraLabel(this, loc, label)
            fontSize = 0.05;
            [colorWhite, colorBlack] = setCameraLabelColor(this);
            this.CameraLabelText = text(loc(1), loc(2), label, 'Parent', getImageAxes(this), ...
                'FontUnits', 'normalized', 'FontSize', fontSize, ...
                'Color',colorBlack,...
                'BackgroundColor', colorWhite,...
                'EdgeColor', colorBlack,...
                'Clipping', 'on', 'HorizontalAlignment', 'right');
        end
        
        %------------------------------------------------------------------
        function plotEpipolarLines(this, I)
            numLines = 10;
            
            X = [1; size(I, 2)];
            X = repmat(X, [1, numLines]);
            
            interval = floor(size(I, 1) / (numLines + 1));
            Y = interval:interval:size(I, 1)-interval+1;
            Y = [Y; Y];
            
            hAxes = getImageAxes(this);
            line(X, Y, 'Parent', hAxes);
        end
        
        %------------------------------------------------------------------
        function plotDetectedPoints(this, detector, p1, p2, plotOrigin)
            hAxes = getImageAxes(this);
            
            if plotOrigin
                pointsToPlot = [p1; p2];
            else
                pointsToPlot = [p1(2:end, :); p2(2:end, :)];
            end

            if isa(detector, 'vision.internal.calibration.webTool.CircleGridDetectorImpl')
                marker = 'gx';
            else
                marker = 'go';
            end
            plot(hAxes, pointsToPlot(:,1), pointsToPlot(:,2), marker, 'LineWidth', 1, 'MarkerSize', 10);
            
            detectedLegend = vision.getMessage('vision:caltool:DetectedLegend');
            this.LegendEntries = {detectedLegend};
        end
        
        %------------------------------------------------------------------
        function drawCoordinateAxes(this, detector, p1, p2)
            hAxes = getImageAxes(this);
            import vision.internal.calibration.tool.*;
            if this.IsImage1Valid
                drawCoordinateAxes(detector, hAxes, p1);
            end
            
            if this.IsImage2Valid
                drawCoordinateAxes(detector, hAxes, p2);
            end
            
            if (this.IsImage1Valid || this.IsImage2Valid)
                originLegend = vision.getMessage('vision:caltool:OriginLegend');
                this.LegendEntries{end+1} = originLegend;
            end
        end
        
        %------------------------------------------------------------------
        function plotReprojectedPoints(this, stereoParams, boardIndex, offset)
            
            if this.IsImage1Valid
                p1 = stereoParams.CameraParameters1.ReprojectedPoints(:, :, boardIndex);
            else
                p1 = zeros(0, 2);
            end
            
            if this.IsImage2Valid
                p2 = stereoParams.CameraParameters2.ReprojectedPoints(:, :, boardIndex);
                p2(:, 1) = p2(:, 1) + offset;
            else
                p2 = zeros(0, 2);
            end
            
            if this.IsImage1Valid || this.IsImage2Valid
                hAxes = getImageAxes(this);
                pointsToPlot = [p1; p2];
                plot(hAxes, pointsToPlot(:,1), pointsToPlot(:,2),'r+','LineWidth', 1,'MarkerSize', 8);
                
                reprojectedLegend = vision.getMessage('vision:caltool:ReprojectedLegend');
                this.LegendEntries{end+1} = reprojectedLegend;
            end
        end
    end

    %Theme specific methods
    methods
        %------------------------------------------------------------------
        function color = setTextColor(this)
             color = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-tertiary');
        end

        function [semWhite, semBlack] = setCameraLabelColor(this)
            semWhite = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-tertiary');
            semBlack = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-gutter-currentExecutingLine');
        end

        function color = setMessagePaneColor(this)
            color = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-notificationBanner');
        end

         function setDisplayColors(this)
            color = setTextColor(this);
            [colorWhite, colorBlack] = setCameraLabelColor(this);
            colorYellow = setMessagePaneColor(this);
            if ~isempty(this.MissingImageText) && isvalid(this.MissingImageText)
                this.MissingImageText.Color = color;
            end
            if ~isempty(this.CameraLabelText) && isvalid(this.CameraLabelText)
                this.CameraLabelText.Color = colorWhite;
                this.CameraLabelText.EdgeColor = colorBlack;
                this.CameraLabelText.BackgroundColor = colorBlack;
            end
            if ~isempty(this.MessagePane) && isvalid(this.MessagePane)
                this.MessagePane.BackgroundColor = colorYellow;
            end
        end
    end
end

