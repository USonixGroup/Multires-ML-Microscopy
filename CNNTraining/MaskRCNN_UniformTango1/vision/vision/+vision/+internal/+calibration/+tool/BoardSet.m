% BoardSet Stores all information about checkerboard images
%
%    This class contains the checkerboard related properties and
%    implementation of the abstract methods related to checkerboard
%    detection.

% Copyright 2012-2021 The MathWorks, Inc.

classdef BoardSet < vision.internal.calibration.tool.PatternSet
    
    properties
        BoardSize
    end
    
    properties (Dependent)  
        SquareSize
        
        IsDistortionHigh
    end
    
    %----------------------------------------------------------------------
    methods (Access=public)
        
        function this = BoardSet(varargin)
            this@vision.internal.calibration.tool.PatternSet(varargin);
            
            if ~isempty(varargin)
                if nargin == 3
                    parent = [];
                else
                    parent = varargin{4};
                end
                
                % Detect checkerboard points
                delIdx = this.detectPointsForNewSession(parent);
                this.deleteRejectedPatterns(delIdx);
                
                % Derive the world points from board dimensions
                this.WorldPoints = generateWorldPoints(this.PatternDetector);
            end
        end
        
    end
    
    %----------------------------------------------------------------------
    % Implement abstract methods from the parent class
    %----------------------------------------------------------------------
    methods   
        
        %------------------------------------------------------------------  
        function numDuplicates = addPatterns(this, fileNames, parent)
            
            if nargin == 2
                parent = [];
            end
            
            % check image consistency
            this.ImageSize = this.checkImageSizes(fileNames);
            
            % proceed with adding images
            numOldImages = this.NumPatterns;
            numNewImages = size(fileNames, 2);
            
            titleId = 'vision:calibrate:AnalyzingImagesTitle';
            messageId = 'vision:calibrate:detectCheckerboardWaitbar';
            tag = 'CheckerboardDetectionProgressBar';
            waitBar = ...
                vision.internal.uitools.ProgressBar(numNewImages, messageId, titleId, tag, parent);
            
            numDuplicates = 0;
            this.LastNonDetectedPathNames = cell(size(fileNames, 1), 0);

            % Get current warning state
            warnState = warning;
            
            % Disable asymmetric board warning since boards are being added
            % to existing session
            warning('off','vision:calibrate:boardShouldBeAsymmetric');
            currBoardSize = this.BoardSize;
            
            for i = 1:numNewImages
                if waitBar.Canceled
                    rollback(this, numOldImages);
                    error(message('vision:uitools:LoadingCanceledByUser'));
                end
                
                if this.isDuplicateFileName(fileNames(:, i))
                    numDuplicates = numDuplicates + 1;
                    continue;
                end
                
                if this.IsStereo
                    % Partial checkerboards are not currently supported for
                    % stereo images.
                    points = detectPatternPoints(this.PatternDetector, ...
                        fileNames{1, i}, fileNames{2, i},...
                        'ProgressBarParent', parent);
                    
                    boardSize = this.PatternDetector.BoardSize;
                else
                    points = detectPatternPoints(this.PatternDetector, fileNames{i},...
                        'ProgressBarParent', parent);

                    boardSize = this.PatternDetector.BoardSize;

                    if ~isempty(points)
                        padSize = currBoardSize - this.PatternDetector.BoardSize;
                        
                        % Pad zeros if the detected board is smaller than the
                        % board in the current session along both dimensions.
                        if all(padSize >= 0)
                            points = this.padPartialBoards(padSize, points, this.PatternDetector.BoardSize);
                            boardSize = currBoardSize;
                        else
                            % Retry detection without partial boards
                            points = detectPatternPoints(this.PatternDetector, fileNames{i}, ...
                                'ProgressBarParent', parent,...
                                'PartialDetections', false);

                            boardSize = this.PatternDetector.BoardSize;
                            
                            padSize = this.BoardSize - boardSize;
                            
                            if all(padSize >= 0)
                                points = this.padPartialBoards(padSize, points, boardSize);
                                boardSize = currBoardSize;
                            end
                        end
                    end
                end
                
                if isequal(boardSize, currBoardSize)
                    this.addOnePattern(fileNames(:, i), points);
                else
                    this.LastNonDetectedPathNames(:, end+1) = fileNames(:, i);
                end
                waitBar.update();
            end
            
            this.BoardSize = currBoardSize;
            this.PatternDetector.BoardSize = currBoardSize;
            
            % Reset warning state
            warning(warnState);
        end
        
        %------------------------------------------------------------------
        function pattern = getPattern(this, patternIdx)
            pattern.boardSize = this.BoardSize;
            pattern.fileName = this.FullPathNames(:, patternIdx);
            pattern.label = this.PatternLabels{patternIdx};
            pattern.detectedPoints = this.PatternPoints(:,:,patternIdx, :);
            pattern.detector = this.PatternDetector;
        end
        
        %------------------------------------------------------------------
        function delIdx = detectPointsForNewSession(this, parent)
            if this.IsStereo
                [boardPoints, goodBoardIdx] = detectPatternPoints(this.PatternDetector, ...
                    this.FullPathNames(1, :), this.FullPathNames(2, :),...
                    'ProgressBarParent', parent,...
                    'ShowProgressBar', true);

                boardSize = this.PatternDetector.BoardSize;

                % Partial checkerboards are not currently supported for
                % stereo images.
                if ~isempty(boardPoints)
                    this.PartialPatterns = false(size(boardPoints,3), 1);
                end
            else
                [boardPoints, goodBoardIdx] = ...
                    detectPatternPoints(this.PatternDetector, this.FullPathNames,...
                    'ProgressBarParent', parent,...
                    'ShowProgressBar', true);

                boardSize = this.PatternDetector.BoardSize;
                
                if ~isempty(boardPoints)
                    this.PartialPatterns = false(size(boardPoints,3), 1);
                    
                    % Set the partial board indices
                    [~, partialBoardsIdx] = find(isnan(boardPoints(:,1,:)));
                    this.PartialPatterns(unique(partialBoardsIdx)) = true;
                end
            end
            
            if isempty(boardPoints)
                error(message('vision:caltool:zeroBoards'));
            end
            
            delIdx = ~goodBoardIdx;
            this.PatternPoints = boardPoints;
            this.BoardSize = boardSize;
        end
        
    end
    
    methods
        
        %------------------------------------------------------------------
        function points = padPartialBoards(~, padSize, points, boardSize)
            
            % Pad zeros assuming the upper left corner point as the origin.
            % If this is missing (not visible/detected), the actual image
            % location of the origin will have to be determined after
            % camera parameter estimation
            currBoardX = reshape(points(:,1), boardSize - 1);
            currBoardX = padarray(currBoardX, padSize, NaN, 'post');
            
            currBoardY = reshape(points(:,2), boardSize - 1);
            currBoardY = padarray(currBoardY, padSize, NaN, 'post');
            
            points = [currBoardX(:), currBoardY(:)];
        end
        
        %------------------------------------------------------------------
        function squareSize = get.SquareSize(this)
            squareSize = this.PatternDetector.SquareSize;
        end
        
        %------------------------------------------------------------------
        function distortion = get.IsDistortionHigh(this)
            distortion = this.PatternDetector.IsDistortionHigh;
        end
        
    end
    
    %----------------------------------------------------------------------
    methods (Static, Hidden)
       
        function this = loadobj(that)
            % disable the mustReturnObject warning  to avoid warning popup
            % The mismatch occurred due to removal of java objects from
            % 'vision.internal.calibration.tool.BoardSet' with older version
            warning('off','MATLAB:class:mustReturnObject');
            
            % Custom detector support was added in 21b. Handle earlier
            % versions of the BoardSet objects
            if ~isa(that, 'vision.internal.calibration.tool.PatternSet')
                this = vision.internal.calibration.tool.BoardSet;

                if size(that.FullPathNames, 1) > 1 % Stereo case
                    this.PatternDetector = vision.calibration.stereo.CheckerboardDetector;
                    this.PatternDetectorFile = 'vision.calibration.stereo.CheckerboardDetector';
                else
                    this.PatternDetector = vision.calibration.monocular.CheckerboardDetector;
                    this.PatternDetectorFile = 'vision.calibration.monocular.CheckerboardDetector';
                end

                this.PatternDetector.BoardSize = that.BoardSize;
                this.PatternDetector.SquareSize = that.SquareSize;
                this.BoardSize = that.BoardSize;
                this.Units = that.Units;
                
                % High distortion and partial detection mode was introduced
                % in 21a. Handle this for previous versions. High
                % distortion and partial detection modes are not supported
                % for stereo.
                if ~size(that.FullPathNames, 1) > 1
                    if isfield(that, 'IsDistortionHigh')
                        this.PatternDetector.IsDistortionHigh = that.IsDistortionHigh;
                    end
                    
                    if isfield(that, 'PartialBoards')
                        this.PartialPatterns = that.PartialBoards;
                    end
                end
                
                this.FullPathNames = that.FullPathNames;
                
                if isfield(that, 'ImageSize')
                    this.ImageSize = that.ImageSize;
                end
                
                this.WorldPoints   = that.WorldPoints;
                this.LastNonDetectedPathNames = that.LastNonDetectedPathNames;
                this.PatternPoints = that.BoardPoints;
                this.PatternLabels = that.BoardLabels;
                this.NumPatterns   = that.NumBoards;
              
            else
                this = that;
            end
            
        end
        
    end
    
end