classdef CircleGridDetector < vision.internal.calibration.CalibrationPatternDetector
% vision.internal.calibration.CircleGridDetector implements the
% detectCircleGridPoints function with the help of its abstract base
% class, vision.internal.calibration.CalibrationPatternDetector.

% Copyright 2024 The MathWorks, Inc.

    % Pattern properties
    properties
        PatternDims
        PatternType
        CircleColor
        DetectorParams
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
        function this = CircleGridDetector(detectorName, varargin)
            narginchk(3,12)
            
            this = this@vision.internal.calibration.CalibrationPatternDetector(...
                detectorName, varargin{1:2});

            validateAndParseInputs(this, varargin{:});
        end

        %-----------------------------------------------------------------------
        function populateDetectorParams(this)
            this.DetectorParams = blobDetectorParameters(this);
            
            this.DetectorParams.IsSymmetric = this.PatternType == "symmetric";
            if this.CircleColor == "black"
                this.DetectorParams.CircleColor = 0;
            elseif this.CircleColor == "white"
                this.DetectorParams.CircleColor = 255;
            end
        end
    end

    %---------------------------------------------------------------------------
    % Define abstract methods from base class
    %---------------------------------------------------------------------------
    methods
        %-----------------------------------------------------------------------
        function [imagePoints, isFound] = detectInOneImage(this, I, ~)
            % detectInOneImage detects a circle grid pattern in an image
            % using an algorithm based on hierarchical clustering. If this
            % algorithm fails, another algorithm based on k-means clustering 
            % is used. Typically, hierarchical clustering algorithm fails if 
            % there is significant background clutter near the pattern in the 
            % image.

            if ischar(I)
                I = imread(I);
            end
            
            params = this.DetectorParams;
            params.HierarchicalClustering = true;
            [imagePoints, isFound] = detectCircleGrid(I, this.PatternDims, params);    
            if ~isFound
                % Use k-means clustering if a circle grid is not found in the image.
                params.HierarchicalClustering = false;
                [imagePoints, isFound] = detectCircleGrid(I, this.PatternDims, params);
            end
        
            if isFound
                % Flip the axes to change the left-handed pattern coordinate  
                % system to a right-handed pattern coordinate system.
                imagePoints = this.flipAxes(imagePoints, size(I, 1:2));

                % LSQCURVEFIT requires output to be double datatype for calibration.
                imagePoints = double(imagePoints);
            else
                imagePoints = [];
            end
        end

        %-------------------------------------------------------------------------------
        function waitBar = createProgressBar(this, numImages)
            titleId = 'vision:calibrate:AnalyzingImagesTitle';
            messageId = 'vision:calibrate:detectCircleGridWaitbar';
            tag = 'CircleGridDetectionProgressBar';
            waitBar = vision.internal.uitools.ProgressBar(numImages, messageId, ...
                titleId, tag, this.ProgressBarParent);
        end
    end

    methods(Access=private)
        %-------------------------------------------------------------------------------
        % Helper function to change the left-handed coordinate system to right-handed
        % coordinate system. It mirrors the order of detected keypoints about the x-axis
        % of the pattern coordinate system, which changes the keypoint chosen as the
        % origin.
        %-------------------------------------------------------------------------------
        function imagePoints = flipAxes(this, imagePoints, imageSize)
            
            numPatternRows = this.PatternDims(1);
            numPatternCols = this.PatternDims(2);
        
            patternCoordsX = reshape(imagePoints(:,1), [numPatternRows, numPatternCols]);
            patternCoordsY = reshape(imagePoints(:,2), [numPatternRows, numPatternCols]);
            patternCoords  = cat(3, patternCoordsX, patternCoordsY);
            patternCoords  = flip(patternCoords, 2);    
            
            patternCoordsX = reshape(patternCoords(:,:,1), [numPatternRows*numPatternCols, 1]);
            patternCoordsY = reshape(patternCoords(:,:,2), [numPatternRows*numPatternCols, 1]);
            imagePoints = [patternCoordsX, patternCoordsY];
        
            if this.PatternType == "symmetric"
                % Symmetric circle grid can have two possible origins in any orientation
                % of the board. The origin in top left half of the image is selected.
                imagePoints = pickOrigin(imagePoints, imageSize);
            end

            %-------------------------------------------------------------------------------
            % Helper function to consistently pick the same origin in a symmetric circle
            % grid. The candidate origin that is on the top-left upper triangle of the
            % image must be picked while maintaining the right-handedness of the pattern
            % coordinate system.
            %-------------------------------------------------------------------------------
            function imagePoints = pickOrigin(imagePoints, imageSize)
            
                origin = imagePoints(1,:);
                if  origin(1) > imageSize(2)/2 || origin(2) > imageSize(1)/2
                    % Change the origin if it does not lie in the top left.
                    imagePoints = imagePoints(end:-1:1,:);
                end
            end
        end

        
    end

    methods
        %-------------------------------------------------------------------------------
        % Input validator.
        % Inputs: 
        %         I, patternDims, NVPs (monocular syntax)
        %         I1, I2, patternDims, NVPs (stereo syntax)
        %-------------------------------------------------------------------------------
        function validateAndParseInputs(this, varargin)
            
            idx = 2 + this.IsStereo;

            this.PatternDims = varargin{idx};
            validatePatternDims(this);

            nvPairs = varargin(idx+1:end);
            validateAndParseNVPairs(this, nvPairs{:});

            populateDetectorParams(this);
            
            % Error if inputs are for a 180-degrees rotation variant asymmetric
            % circle grid.
            areBothDimsEven = all(mod(this.PatternDims, 2) == 0);
            vision.internal.errorIf(this.PatternType == "asymmetric" && areBothDimsEven,...
                'vision:calibrate:unsupportedPatternDims')
            
            % Error if symmetric circle grid pattern is used with stereo camera.
            vision.internal.errorIf(this.PatternType == "symmetric" && this.IsStereo,...
                'vision:calibrate:unsupportedPattern')
        end

        %-------------------------------------------------------------------------------
        function validateAndParseNVPairs(this, options)
            arguments
                this
                options.PatternType (1,1) string = "asymmetric"
                options.CircleColor (1,1) string = "black"
                options.ShowProgressBar (1,1) {mustBeA(options.ShowProgressBar, 'logical')} = false
                options.ProgressBarParent = []
            end
           
            this.PatternType = validatestring(options.PatternType, ...
                                  {'symmetric','asymmetric'}, this.DetectorName, 'PatternType');
            this.CircleColor = validatestring(options.CircleColor, ...
                                  {'white','black'}, this.DetectorName, 'CircleColor');
            
            this.ShowProgressBar = options.ShowProgressBar;
            this.ProgressBarParent = options.ProgressBarParent;
        end
        
        %-------------------------------------------------------------------------------
        function validatePatternDims(this)
           validateattributes(this.PatternDims, {'numeric'},...
                {'nonempty', 'vector', 'numel', 2, 'integer', 'positive', '>=', 3}, ...
                this.DetectorName, 'patternDims');    
        end      
    end
end

%-------------------------------------------------------------------------------
function params = blobDetectorParameters(this)
    % Hyperparameter values for blob detector that were empirically tuned to
    % work for most calibration images for cameras upto 86° field of view.
    
    params.CircleColor = 0; % black by default.
    params.MinThreshold = 8;
    params.MaxThreshold = 255;
    params.MinCircularity = 0.7;
    params.MinConvexity = 0.87;
    params.MinInertiaRatio = 0.01;
    params.MinArea = 50; % in pixels.

    % Based on empirical analysis, a max area of 1e5 works for most images,
    % but decreasing the max area for smaller images improves the
    % robustness of the detection.
    imageArea = numel(this.Images1);
    imageAreaThreshold = 2000000;
    if imageArea > imageAreaThreshold
        params.MaxArea = 1e5; % in pixels.
    else
        params.MaxArea = 1e4; % in pixels.
    end
end