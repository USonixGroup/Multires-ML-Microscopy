classdef SingleCalibrationImageDisplay < vision.internal.calibration.webTool.CalibrationImageDisplay
    % SingleCalibrationImageDisplay Class that encapsulates the main image display
    %   in the Camera Calibrator App
    %
    % SingleCalibrationImageDisplay methods:
    %    drawBoard  - display the calibration image with annotations
    
    
    % Copyright 2014-2024 The MathWorks, Inc.
    
    properties
        CalibrationImageDocGroup
        MissingImgText
    end
    
    methods
        function this = SingleCalibrationImageDisplay(options)
            this = this@vision.internal.calibration.webTool.CalibrationImageDisplay(options);
        end
        
        %------------------------------------------------------------------
        function drawBoard(this, board, boardIdx, cameraParams)
            % drawBoard Display the calibration image with annotations.
            %   drawBoard(obj, board, boardIdx, cameraParams) renders the
            %   calibration image, detected points, and board axes labels. If
            %   cameraParams input is non-empty, the method also plots the
            %   reprojected points.
            %
            %   Inputs:
            %   -------
            %   obj          - A SingleCalibrationImageDisplay object
            %
            %   board        - A struct containing the following fields:
            %                  boardSize      - [rows,cols]
            %                  fileName       - 1-element cell array containing image file
            %                  label          - title of the image axes
            %                  detectedPoints - M-by-2 array of x-y coordinates
            %
            %   boardIdx     - Index of this image/board
            %
            %   cameraParams - A cameraParameters object
            if ~ishandle(this.Figure)
                return;
            end
            
            drawBoardImpl(this, board, boardIdx, cameraParams);
        end
        
        %------------------------------------------------------------------
        function drawUndistortedBoard(this, board, camParams, scaleFactor)
            try
                I = imread(board.fileName{1});
                if isa(camParams, 'fisheyeParameters')
                    I = undistortFisheyeImage(I, camParams.Intrinsics, ...
                        'ScaleFactor', scaleFactor);
                else
                    I = undistortImage(I, camParams);
                end
                
                drawImage(this, I);
                setTitle(this, board.label);
                holdOff(this);
            catch
                handleMissingImage(this, board.label);
            end
        end
    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function drawBoardImpl(this, board, boardIdx, camParams)
            try
                I = imread(board.fileName{1});
                
                makeHandleVisible(this);
                
                drawImage(this, I);
                
                plotDetectedPoints(this, board);
                
                if ~isempty(camParams) && (boardIdx(1) <= camParams.NumPatterns)
                    % draw the reprojection errors only if they are available
                    % for the particular board; they may not be available if a
                    % new set of boards was added to a session which already
                    % involved a successful calibration
                    plotReprojectedPoints(this, board, boardIdx, camParams);
                end
                
                showLegend(this);
                setTitle(this, board.label);
                holdOff(this);
                set(this.Figure,'HandleVisibility','callback');
                
            catch
                handleMissingImage(this, board.label);
            end
        end
        
        %------------------------------------------------------------------
        function handleMissingImage(this, fileName)
            I = zeros(480, 640);
            makeHandleVisible(this);
            drawImage(this, I);
            hAxes = getImageAxes(this);
            color = this.setTextColor;
            this.MissingImgText = text(320, 240, ['Cannot read ', fileName], ...
                'Color', color, 'Parent', hAxes, 'HorizontalAlignment', 'center');
            holdOff(this);
            set(this.Figure,'HandleVisibility','callback');
        end
        
        %------------------------------------------------------------------
        function plotDetectedPoints(this, board)
            hAxes = getImageAxes(this);
            p = board.detectedPoints;
            
            if isa(board.detector, 'vision.calibration.monocular.CheckerboardDetector')...
                    || isa(board.detector, 'vision.internal.calibration.monocular.CheckerboardDetector')
                plot(hAxes, p(2:end,1), p(2:end,2), 'go', 'LineWidth', 1, 'MarkerSize', 10);
            elseif isa(board.detector, 'vision.internal.calibration.webTool.CircleGridDetectorImpl')
                plot(hAxes, p(:,1), p(:,2), 'gx', 'LineWidth', 1, 'MarkerSize', 10);
            else
                plot(hAxes, p(:,1), p(:,2), 'go', 'LineWidth', 1, 'MarkerSize', 10);
            end
            
            % add a legend
            %-------------
            detectedLegend = vision.getMessage('vision:caltool:DetectedLegend');
            this.LegendEntries = {detectedLegend};
            
            vision.internal.calibration.tool.drawCoordinateAxes(board.detector, hAxes, p);
            
            % Add origin legend for patterns.
            originLegend = vision.getMessage('vision:caltool:OriginLegend');
            this.LegendEntries{end+1} = originLegend;
        end
        
        %------------------------------------------------------------------
        function plotReprojectedPoints(this, board, boardIndex, camParams)
            hAxes = getImageAxes(this);
            reprojectedLegend = vision.getMessage('vision:caltool:ReprojectedLegend');
            this.LegendEntries{end+1} = reprojectedLegend;
            p = board.detectedPoints;
            rp = p + camParams.ReprojectionErrors(:,:,boardIndex);
            plot(hAxes, rp(:,1),rp(:,2),'r+','LineWidth', ...
                1,'MarkerSize', 8);
        end
    end
   
    % Theme specific functions
    methods

        %------------------------------------------------------------------
        function color = setTextColor(this)
             color = matlab.graphics.internal.themes.getAttributeValue...
                (this.Theme, '--mw-backgroundColor-tertiary');
        end

        %------------------------------------------------------------------
        function setDisplayColors(this)
            color = setTextColor(this);
            if ~isempty(this.MissingImgText) && isvalid(this.MissingImgText)
                this.MissingImgText.Color = color;
            end
        end
    end
end
