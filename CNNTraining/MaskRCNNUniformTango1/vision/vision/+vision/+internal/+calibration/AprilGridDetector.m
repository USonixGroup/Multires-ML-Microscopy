classdef AprilGridDetector < vision.internal.calibration.CalibrationPatternDetector
% vision.internal.calibration.AprilGridDetector implements the
% detectAprilGridPoints function with the help of its abstract base
% class, vision.internal.calibration.CalibrationPatternDetector.

% Copyright 2024 The MathWorks, Inc.

    % Pattern detector properties
    properties
        PatternDims
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
        function this = AprilGridDetector(detectorName, varargin)
            narginchk(3,16)
            
            this = this@vision.internal.calibration.CalibrationPatternDetector(...
                detectorName, varargin{1:2});

            [this.DetectorParams, this.ShowProgressBar, this.ProgressBarParent] = ...
                vision.internal.inputValidation.parseDetectAprilGridInputs(...
                this.IsStereo, varargin{:});
        end
    end

    %---------------------------------------------------------------------------
    % Define abstract methods from base class
    %---------------------------------------------------------------------------
    methods
        %-----------------------------------------------------------------------
        function [imagePoints, isFound] = detectInOneImage(this, I, ~)
            
            if ischar(I)
                I = imread(I);
            end

            points = detectAprilGrid(I, this.DetectorParams);
            
            isFound = ~isempty(points);
            if isFound
                % Flip the axes to change the left-handed pattern coordinate  
                % system to a right-handed pattern coordinate system.
                points = this.flipAxes(points);

                % LSQCURVEFIT requires output to be double datatype for calibration.
                imagePoints = double(points);
            else
                imagePoints = [];
            end
        end

        %-----------------------------------------------------------------------
        function waitBar = createProgressBar(this, numImages)
            titleId = 'vision:calibrate:AnalyzingImagesTitle';
            messageId = 'vision:calibrate:detectAprilGridWaitbar';
            tag = 'AprilGridDetectionProgressBar';
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
            [numImages2, refSize2] = this.imageDims(input2);
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
        % which changes the keypoint chosen as the origin and ensures the
        % y-axis falls on the shortest dimension of the pattern which is the 
        % CVT calibration convention.
        %-----------------------------------------------------------------------
        function imagePoints = flipAxes(this, imagePoints)
            
            numPatternRows = this.DetectorParams.PatternDims(1)*2;
            numPatternCols = this.DetectorParams.PatternDims(2)*2;

            % 3p tools generate the AprilGrid pattern with tag ids in the
            % right handed coordinate system when numCols < numRows. No
            % need to flip the pattern in that case.
            if numPatternCols >= numPatternRows
                
                patternCoordsX = reshape(imagePoints(:,1), [numPatternRows, numPatternCols]);
                patternCoordsY = reshape(imagePoints(:,2), [numPatternRows, numPatternCols]);
                patternCoords  = cat(3, patternCoordsX, patternCoordsY);
                
                % Flip the board along the longer side to maintain
                % right-handedness.
                patternCoords  = flip(patternCoords, 1);
            
                patternCoordsX = patternCoords(:,:,1);
                patternCoordsY = patternCoords(:,:,2);
                patternCoordsX = reshape(patternCoordsX, [numPatternRows*numPatternCols, 1]);
                patternCoordsY = reshape(patternCoordsY, [numPatternRows*numPatternCols, 1]);
                imagePoints = [patternCoordsX, patternCoordsY];
            end
        end
    end

end

%--------------------------------------------------------------------------
function [keypoints, keypointIds] = detectAprilGrid(I, params)
    
    % Read AprilTags.
    detectorParams.NumBorderBits = params.NumBorderBits;
    detectorParams.GaussianSigma = params.GaussianSigma;
    detectorParams.DecimationFactor = params.DecimationFactor;
    
    [ids, locs] = vision.internal.calibration.readAprilTag(I, params.TagFamily, detectorParams);
    
    % Filter ids.
    invalidIds = ids < params.MinTagID | ids > params.MaxTagID;
    locs(:,:,invalidIds) = [];
    ids(invalidIds) = [];

    % Normalize ids.
    ids = ids - params.MinTagID;

    % Convert tag points to grid points.
    if ~isempty(locs)
        [keypoints, keypointIds] = tagToGridPoints(locs, ids, params.PatternDims);
    else
        keypoints = [];
        keypointIds = [];
    end
end

%--------------------------------------------------------------------------
% Reorder the ApriltTag corners to form the AprilGrid keypoints. 
% Inputs: locs - 4-by-2-M | ids - M-by-1 | patternDims - [dim1, dim2]
% Outputs:  imagePoints - N-by-2 | keypointIds - N-by-1 where N = dim1*dim2*4
%--------------------------------------------------------------------------
function [imagePoints, keypointIds] = tagToGridPoints(locs, ids, patternDims)

    numKeypoints = patternDims(1)*patternDims(2)*4;
    imagePoints = nan(numKeypoints, 2);
    keypointIds = nan(numKeypoints, 1);

    for i = 1:length(ids)
        
        ind = findKeypointsIndices(ids(i), patternDims);

        imagePoints(ind(1),:) = locs(1,:,i);
        imagePoints(ind(2),:) = locs(2,:,i);
        imagePoints(ind(3),:) = locs(3,:,i);
        imagePoints(ind(4),:) = locs(4,:,i);
        keypointIds(ind(1)) = 4*ids(i);
        keypointIds(ind(2)) = 4*ids(i)+1;
        keypointIds(ind(3)) = 4*ids(i)+2;
        keypointIds(ind(4)) = 4*ids(i)+3;
    end
end

%--------------------------------------------------------------------------
% Helper to find the indices of the AprilGrid keypoints given their
% corresponding AprilTag ID and the AprilGrid dimensions. Each AprilTag
% contributes to 4 AprilGrid keypoints. So, the output ind is a 4-by-1
% vector corresponding to the indices of the 4 corners in the AprilGrid
% keypoint matrix.
%--------------------------------------------------------------------------
function ind = findKeypointsIndices(id, dims)
    
    numRows = dims(1);
    numCols = dims(2);
    [col, row] = ind2sub(dims([2 1]), id+1);
    
    % Connect the points along the shortest dimension which is the y-axis.
    connectAlongColums = dims(2) < dims(1);
    if connectAlongColums
        totalNumPointsBefore = (row-1)*numCols*4;
        ptIndexInCurrentRow = 2*(col-1) + 1;
        bottomLeftPtInd = totalNumPointsBefore + ptIndexInCurrentRow;

        % topLeft point is offset by one row from the bottomLeft in the same ptIndex.
        topLeftPtInd = bottomLeftPtInd + 2*numCols;
        
        % readAprilTag returns the points in the following order:
        % [bottomLeft, bottomRight, topRight, topLeft]. We ideally want,
        % [bottomLeft, bottomRight, topLeft, topRight].
        ind = [bottomLeftPtInd bottomLeftPtInd+1 topLeftPtInd+1 topLeftPtInd];
    else
        totalNumPointsBefore = (col-1)*numRows*4;
        ptIndexInCurrentColumn = 2*(row-1) + 1;
        bottomLeftPtInd = totalNumPointsBefore + ptIndexInCurrentColumn;

        % bottomRight point is offset by one column from the bottomLeft in the same ptIndex.
        bottomRightPtInd = bottomLeftPtInd + 2*numRows;
        
        ind = [bottomLeftPtInd bottomRightPtInd bottomRightPtInd+1 bottomLeftPtInd+1];
    end
end