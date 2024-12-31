% CustomPatternSet Stores all information about calibration images for a
% custom pattern
%
%    This class contains properties for a custom user-defined calibration
%    pattern. The detection function and world point generation
%    functionalities are provided by the user

% Copyright 2021-2024 The MathWorks, Inc.

classdef CustomPatternSet < vision.internal.calibration.tool.PatternSet
    
    %----------------------------------------------------------------------
    methods (Access=public)
        
        function this = CustomPatternSet(varargin)

            this@vision.internal.calibration.tool.PatternSet(varargin);

            % Detect calibration pattern points in images
            delIdx = this.detectPointsForNewSession();
            this.deleteRejectedPatterns(delIdx);
            
            % Derive the world points for calibration pattern
            this.WorldPoints = generateWorldPoints(this.PatternDetector); 
            
            % Validate generated world points
            validateWorldPoints(this);
        end
            
        %------------------------------------------------------------------
        function reset(this)
            % Call parent class reset
            reset@vision.internal.calibration.tool.PatternSet(this);
        end
        
    end % public methods
    
    %----------------------------------------------------------------------
    % Implement abstract methods from the parent class
    %----------------------------------------------------------------------
    methods         
        
        %------------------------------------------------------------------
        function numDuplicates = addPatterns(this, fileNames, ~)
           
            % check image consistency
            this.ImageSize = this.checkImageSizes(fileNames);
            
            % proceed with adding images
            numNewImages = size(fileNames, 2);
            
            numDuplicates = 0;
            this.LastNonDetectedPathNames = cell(size(fileNames, 1), 0);
            
            for i = 1:numNewImages
                if this.isDuplicateFileName(fileNames(:, i))
                    numDuplicates = numDuplicates + 1;
                    continue;
                end
                
                if this.IsStereo
                    points = detectPatternPoints(this.PatternDetector, ...
                        fileNames{1, i}, fileNames{2, i});


                    % Validate detected keypoints
                    validatePatternPointsSingle(this, points);
                else
                    points = detectPatternPoints(this.PatternDetector, fileNames{i});
                    
                    % Validate detected keypoints
                    validatePatternPointsSingle(this, points);
                end
                
                if isequal(size(points, 1), size(this.PatternPoints,1))
                    this.addOnePattern(fileNames(:, i), points);
                else
                    this.LastNonDetectedPathNames(:, end+1) = fileNames(:, i);
                end
            end
        end
        
        %------------------------------------------------------------------
        function pattern = getPattern(this, patternIdx)
            pattern.fileName = this.FullPathNames(:, patternIdx);
            pattern.label = this.PatternLabels{patternIdx};
            pattern.detectedPoints = this.PatternPoints(:,:,patternIdx, :);
            pattern.detector = this.PatternDetector;
        end
        
        %------------------------------------------------------------------
        function delIdx = detectPointsForNewSession(this)
            if this.IsStereo

                [patternPoints, goodBoardIdx] = detectPatternPoints(this.PatternDetector, ...
                    this.FullPathNames(1, :), this.FullPathNames(2, :));

                % Validate detected keypoints
                validatePatternPointsMultiple(this, patternPoints, goodBoardIdx);

                if ~isempty(patternPoints)
                    this.PartialPatterns = false(size(patternPoints,3), 1);
                    
                    % Set the partial board indices. In stereo case, an
                    % image pair is considered to contain a partial board
                    % if any of the keypoints are not detected in either of
                    % those images.
                    [~, partialBoardsIdx] = find(any(isnan(patternPoints(:,1,:,:)),4));

                    this.PartialPatterns(unique(partialBoardsIdx)) = true;
                end
            else
                [patternPoints, goodBoardIdx] = detectPatternPoints(this.PatternDetector, this.FullPathNames);

                % Validate detected keypoints
                validatePatternPointsMultiple(this, patternPoints, goodBoardIdx);
                
                if ~isempty(patternPoints)
                    this.PartialPatterns = false(size(patternPoints,3), 1);
                    
                    % Set the partial board indices
                    [~, partialBoardsIdx] = find(isnan(patternPoints(:,1,:)));
                    this.PartialPatterns(unique(partialBoardsIdx)) = true;
                end
            end
            
            delIdx = ~goodBoardIdx;
            this.PatternPoints = patternPoints;
        end

    end
    
    methods(Access=private)
        %------------------------------------------------------------------
        function validatePatternPointsMultiple(this, patternPoints, goodBoardIdx)
        
            % goodBoardIdx must be a logical array of size numImages
            if this.IsStereo
                validateattributes(goodBoardIdx, {'numeric','logical'},...
                    {'vector', 'numel', size(this.FullPathNames, 2)}, 'detectPatternPoints','pairsUsed');
            else
                validateattributes(goodBoardIdx, {'numeric','logical'},...
                    {'vector', 'numel', numel(this.FullPathNames)}, 'detectPatternPoints','imagesUsed');
            end
            
            numDetections = nnz(goodBoardIdx);
            
            % patternPoints must be an M-by-2-by-numImages (monocular) or
            % M-by-2-by-numImages-by-2 (stereo) numeric matrix.
            if this.IsStereo
            	expectedSize = [NaN, 2, numDetections, 2];
            else
            	expectedSize = [NaN, 2, numDetections];
            end

            if numDetections==0 && ~isempty(patternPoints)
            	error(message('vision:caltool:NonEmptyBoardPoints'));
            elseif isempty(patternPoints)
            	error(message('vision:caltool:zeroBoards'));
            else
            	validateattributes(patternPoints, {'single','double'},...
            		{'real', 'nonnegative', 'size', expectedSize}, ...
            		'detectPatternPoints','imagePoints');
            end
        end
        
        %------------------------------------------------------------------
        function validatePatternPointsSingle(this, patternPoints)
           if ~isempty(patternPoints)
               if this.IsStereo
                  validateattributes(patternPoints, {'single','double'},...
                        {'real', 'nonnegative', 'finite', 'size', [NaN, 2, 1, 2]},'detectPatternPoints','imagePoints'); 
               else
                  validateattributes(patternPoints, {'single','double'},...
                        {'real', 'nonnegative', 'size', [NaN, 2]},'detectPatternPoints','imagePoints'); 
               end
           end
        end
        
        %------------------------------------------------------------------
        function validateWorldPoints(this)
            % Determine number of points in pattern
            if ~isempty(this.PatternPoints)
                numPoints = size(this.PatternPoints, 1);
            else
                numPoints = NaN;
            end
            
            validateattributes(this.WorldPoints, {'single','double'},...
                {'real', 'nonnegative', 'size', [numPoints, 2]}, 'generateWorldPoints', 'worldPoints');
        end
            
    end
    
end
