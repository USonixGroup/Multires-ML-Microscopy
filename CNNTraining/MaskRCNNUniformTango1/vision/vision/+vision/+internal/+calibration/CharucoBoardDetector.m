classdef CharucoBoardDetector < vision.internal.calibration.CalibrationPatternDetector
% vision.internal.calibration.CharucoBoardDetector implements the
% detectCharucoBoardPoints function with the help of its abstract base
% class, vision.internal.calibration.CalibrationPatternDetector.

% Copyright 2024 The MathWorks, Inc.

    % Pattern detector properties
    properties
        PatternDims
        DetectorParams
        Intrinsics
    end

    % Abstract properties from base class
    properties 
        ShowProgressBar
        ProgressBarParent
    end

    methods
        %-----------------------------------------------------------------------
        % Constructor
        %-----------------------------------------------------------------------
        function this = CharucoBoardDetector(detectorName, varargin)
            narginchk(6,32)
            
            this = this@vision.internal.calibration.CalibrationPatternDetector(...
                detectorName, varargin{1:2});

            [this.DetectorParams, this.Intrinsics, this.ShowProgressBar, this.ProgressBarParent] = ...
                vision.internal.inputValidation.parseDetectCharucoInputs(this.IsStereo, varargin{:});
        end
    end

    %---------------------------------------------------------------------------
    % Define abstract methods from base class
    %---------------------------------------------------------------------------
    methods
        %-----------------------------------------------------------------------
        function [imagePoints, isFound] = detectInOneImage(this, I, varargin)
            
            if ischar(I)
                I = imread(I);
            end

            if nargin > 2
                cameraIdx = varargin{1};
                intrinsics = this.Intrinsics(cameraIdx);
            else
                intrinsics = this.Intrinsics;
            end

            [points, ids] = detectCharucoBoard(I, this.DetectorParams, intrinsics);
            
            isFound = ~isempty(points);
            if isFound
                [ids, idx] = sort(ids);
                points = points(idx,:);

                % Expected number of points in a full board detection can
                % be different from number of points detected during
                % partial board detections.
                expNumPoints = prod(this.DetectorParams.PatternDims-1);
                imagePoints = nan(expNumPoints,2);
                imagePoints(ids+1,:) = points;
        
                % Flip the axes to change the left-handed pattern coordinate 
                % system to a right-handed pattern coordinate system.
                imagePoints = this.flipAxes(imagePoints);

                % LSQCURVEFIT requires output to be double datatype for calibration.
                imagePoints = double(imagePoints);
            else
                imagePoints = [];
            end
        end

        %-----------------------------------------------------------------------
        function waitBar = createProgressBar(this, numImages)
            titleId = 'vision:calibrate:AnalyzingImagesTitle';
            messageId = 'vision:calibrate:detectCharucoBoardWaitbar';
            tag = 'CharucoBoardDetectionProgressBar';
            waitBar = vision.internal.uitools.ProgressBar(numImages, messageId, ...
                titleId, tag, this.ProgressBarParent);
        end
    end

    methods
        %-----------------------------------------------------------------------
        % Override base class detectInMultipleImagePairs implementation
        % with this generalized implementation.
        %-----------------------------------------------------------------------
        function [imagePoints, pairsUsed, userCanceled] = ...
            detectInMultipleImagePairs(this, input1, input2)
            
            [numImages1, refSize1] = this.imageDims(input1);
            [numImages2, refSize2] = this.imageDims(input1);
            assert(numImages1==numImages2)

            % Create progress bar dialog.
            userCanceled = false;
            if this.ShowProgressBar
                waitBar = this.createProgressBar(2*numImages1);
            end
            
            points = cell(numImages1, 1);
            pairsUsed = false(numImages1, 1);
            for i = 1:numImages1       
                if this.ShowProgressBar && waitBar.Canceled
                    imagePoints  = [];
                    pairsUsed    = [];
                    userCanceled = true;
                    return
                end
                
                I1 = this.imageRead(input1, i, refSize1);
                I2 = this.imageRead(input2, i, refSize2);
                [points{i}, pairsUsed(i)] = detectInOneImagePair(this, ...
                    I1, I2); 
                
                if this.ShowProgressBar
                    waitBar.update();
                end
            end
        
            points(~pairsUsed) = [];
            imagePoints = cat(3, points{:});
        end
    end

    methods(Access=private)
        %-----------------------------------------------------------------------
        % Helper function to change the left-handed coordinate system to
        % right-handed coordinate system. It mirrors the order of detected
        % keypoints about the x-axis of the pattern coordinate system,
        % which changes the keypoint chosen as the origin.
        %-----------------------------------------------------------------------
        function imagePoints = flipAxes(this, imagePoints)
            
            % number of rows & cols here correspond to unrotated board's rows and
            % columns in the OpenCV conventions.
            numPatternRows = this.DetectorParams.PatternDims(2)-1;
            numPatternCols = this.DetectorParams.PatternDims(1)-1;
        
            patternCoordsX = reshape(imagePoints(:,1), [numPatternRows, numPatternCols]);
            patternCoordsY = reshape(imagePoints(:,2), [numPatternRows, numPatternCols]);
            patternCoords  = cat(3, patternCoordsX, patternCoordsY);
        
            if numPatternCols < numPatternRows
                % Connect the points in the y-axis direction. (that is the shortest
                % dimension direction.) x-axis is always along the longest
                % dimension of the board. OpenCV generates the ChArUco board with
                % marker ids in the right handed coordinate system. No need to flip
                % the board.
        
                patternCoordsX = patternCoords(:,:,1)';
                patternCoordsY = patternCoords(:,:,2)';
        
            else
                % Flip the board along the longer side to maintain
                % right-handedness.
                patternCoords  = flip(patternCoords, 2);
            
                patternCoordsX = patternCoords(:,:,1);
                patternCoordsY = patternCoords(:,:,2);
            end
        
            patternCoordsX = reshape(patternCoordsX, [numPatternRows*numPatternCols, 1]);
            patternCoordsY = reshape(patternCoordsY, [numPatternRows*numPatternCols, 1]);
            imagePoints = [patternCoordsX, patternCoordsY];
        end
    end
end