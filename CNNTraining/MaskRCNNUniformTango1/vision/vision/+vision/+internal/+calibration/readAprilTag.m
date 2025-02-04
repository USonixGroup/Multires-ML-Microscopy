function [ids, locs, Ithresholded, Icontours, Ivalidcontours, Iquads] = readAprilTag(I, tagFamily, varargin)
% readAprilTag Detect and estimate pose for AprilTag in image
%   This MATLAB function detects AprilTags in the input image I and returns
%   the locations and IDs associated with the tags.
%
%   Syntax:
%   ------
%   [id, locs] = readAprilTag(I, tagFamily, params)
%
%   Input arguments:
%   ---------------
%     I - Input image
%       M-by-N-by-3 truecolor image | M-by-N grayscale image
%     tagFamily - AprilTag family
%       valid AprilTag family name
%     params - see defaultAprilTagDetectorParams function for their definitions.
%       GaussianSigma
%       DecimationFactor
%       NumBorderBits
%       GaussianWindowSize
%       AutoResizeLargeImage
%       DilationWindowSize
%       ErosionWindowSize
%       DetectorThreshold
%       EdgeDilationWindowSize
%       QuadRatioThreshold
%       PolygonalPrecision
%       RefineEdges
%       RefinementWindowSize
%       ZeroZoneWinSize
%       MaxIterations
%       ErrorTolerance
%       DecodeThresholdOffset
%       MarkerSize
%       MinimumDistance
%       HammingThreshold
%       Debug
%
%   Output arguments:
%   ----------------
%   id - tag Ids of size 1xN
%   locs - locations of detected aprilTags in pattern of size 4x2xM
%
% Copyright 2024 The MathWorks, Inc.

narginchk(2,44);

% Validate image
vision.internal.inputValidation.validateImage(I);

% Get detector parameters based on input tagFamily and name-value arguments.
params = parseDetectorParameters(tagFamily, varargin{:});

% Convert to grayscale
if ~ismatrix(I)
    imgGray = rgb2gray(I);
else
    imgGray = I;
end

% Ensure image data is uint8
if ~isa(imgGray, 'uint8')
    imgUint8 = im2uint8(imgGray);
else
    imgUint8 = imgGray;
end

% Call to C++ function
[detections, tagIds, Ithresholded, Icontours, Ivalidcontours, Iquads] = ...
    detectAprilGrid(imgUint8, params);

if numel(tagIds)>0
    % Package output if detections are detected
    tags = packageDetections(detections, tagIds);

    ids = tags.tagId;
    locs = tags.points;
else
    ids = [];
    locs = [];
    return;
end

end

%--------------------------------------------------------------------------
% parseDetectorParameters - parse detector parameters
%--------------------------------------------------------------------------
function params = parseDetectorParameters(tagFamily, varargin)

% Get default values for detector parameters
params = defaultAprilTagDetectorParams(tagFamily);

% Create parser
parser = inputParser;

% Add Name-Value arguments to input parser
addOptional(parser, "GaussianSigma", params.GaussianSigma);
addOptional(parser, "GaussianWindowSize", params.GaussianWindowSize);
addOptional(parser, "AutoResizeLargeImage", params.AutoResizeLargeImage);
addOptional(parser, "DecimationFactor", params.DecimationFactor);
addOptional(parser, "DilationWindowSize", params.DilationWindowSize);
addOptional(parser, "ErosionWindowSize", params.ErosionWindowSize);
addOptional(parser, "DetectorThreshold", params.DetectorThreshold);
addOptional(parser, "EdgeDilationWindowSize", params.EdgeDilationWindowSize);
addOptional(parser, "QuadRatioThreshold", params.QuadRatioThreshold);
addOptional(parser, "PolygonalPrecision", params.PolygonalPrecision);

addOptional(parser, "RefineEdges", params.RefineEdges);
addOptional(parser, "RefinementWindowSize", params.RefinementWindowSize);
addOptional(parser, "ZeroZoneWinSize", params.ZeroZoneWinSize);
addOptional(parser, "MaxIterations", params.MaxIterations);
addOptional(parser, "ErrorTolerance", params.ErrorTolerance);

addOptional(parser, "DecodeThresholdOffset", params.DecodeThresholdOffset);
addOptional(parser, "NumBorderBits", params.NumBorderBits);
addOptional(parser, "MarkerSize", params.MarkerSize);
addOptional(parser, "MinimumDistance", params.MinimumDistance);
addOptional(parser, "HammingThreshold", params.HammingThreshold);

addOptional(parser, "Debug", params.Debug);

% Parse the input arguments
parse(parser,varargin{:});

% Retrieve the parser results
params = parser.Results;
end

%--------------------------------------------------------------------------
% uniqueDetections - Avoid duplication of detections and TagIds
%--------------------------------------------------------------------------
function [detections, tagIds] = uniqueDetections(detections, tagIds)

% Get the unique tag IDs and the indices of all occurrences
[uniqueTagIds, ~, ic] = unique(tagIds, 'stable');

% Preallocate the matrix for the best detections (with minimum area)
bestDetections = zeros(length(uniqueTagIds)*4, 2);

% Loop over each unique tag ID
for i = 1:length(uniqueTagIds)
    % Find all indices of the current tag ID
    currentTagIndices = find(ic == i);

    % Initialize variables to track the minimum area and the index of the detection with the minimum area
    minArea = Inf;
    minAreaIndex = 0;

    % Loop over all detections for the current tag ID
    for j = 1:length(currentTagIndices)
        % Calculate the start and end row indices for the 4 points of the current detection
        startIdx = (currentTagIndices(j) - 1) * 4 + 1;
        endIdx = startIdx + 3;

        % Extract the x and y coordinates of the 4 points
        x = detections(startIdx:endIdx, 1);
        y = detections(startIdx:endIdx, 2);

        % Calculate the area of the quadrilateral
        area = polyarea(x, y);

        % Update the minimum area and index if the current area is smaller
        if area < minArea
            minArea = area;
            minAreaIndex = currentTagIndices(j);
        end
    end

    % Calculate the start and end row indices for the 4 points of the detection with the minimum area
    startIdx = (minAreaIndex - 1) * 4 + 1;
    endIdx = startIdx + 3;

    % Calculate the row indices for the bestDetections matrix
    bestStartIdx = (i - 1) * 4 + 1;
    bestEndIdx = bestStartIdx + 3;

    % Select the 4 x 2 matrix of detections for the tag ID with the minimum area
    bestDetections(bestStartIdx:bestEndIdx, :) = detections(startIdx:endIdx, :);
end

detections = bestDetections;
tagIds = uniqueTagIds;

end

%--------------------------------------------------------------------------
% packageDetections - Package detections and tagIds into a structure
%--------------------------------------------------------------------------
function tags = packageDetections(detections, tagIds)

% Ensure that the number of detections is exactly four times the number of tagIds
assert(mod(size(detections, 1), numel(tagIds)) == 0, 'The number of detections must be a multiple of the number of tag IDs.');

% Keep unique tagIds and detections
[detections, tagIds] = uniqueDetections(detections, tagIds);

% Number of corners per tag (assuming quadrilateral tags)
cornersPerTag = size(detections, 1) / numel(tagIds);

% If detections are 0 then return an empty structure
if isempty(tagIds)
    % Return an empty structure array if there are no valid tagIds
    tags = struct('tagId', zeros(1,0), 'points', zeros(0,0,0));
    return;
end

% Initialize the structure array
N = numel(tagIds);
tags = struct('tagId', zeros(1,N), 'points', zeros(4,2,N));

% Loop through each tagId to create the structure
for i = 1:N
    % Index for the corners of the current tag
    cornerIndices = (i-1)*cornersPerTag + (1:cornersPerTag);

    % Assign the tagId and corresponding points to the structure
    tags.tagId(i) = tagIds(i);
    tags.points(:,:,i) = detections(cornerIndices, :);
end

end

%--------------------------------------------------------------------------
% getSupportedTagFamily - List of tag family and their corresponding
% non-tunable parameters
%   - marker size
%   - minimum distance
%   - hamming distance
%--------------------------------------------------------------------------
function TagFamilyDict = getSupportedTagFamily()
    TagFamilyDict = struct( ...
        'tag36h11',   [6, 11, 3], ...
        'tag25h9',    [5, 9, 2], ...
        'tag25h7',    [5, 7, 2], ...
        'tag16h5',    [4, 5, 1] ...
        );
end

%--------------------------------------------------------------------------
% getTagFamilyParams - Check if valid tag family input is provided
%--------------------------------------------------------------------------
function tagFamilyParams = getTagFamilyParams(tagFamily)
    supportedTagFamilies = getSupportedTagFamily();

    % Check if the provided tagFamily is a valid field in the structure
    if isfield(supportedTagFamilies, tagFamily)
        tagFamilyParams = supportedTagFamilies.(tagFamily);
    else
        error(message('vision:apriltag:invalidAprilTagFamily',tagFamily));
    end

end
%--------------------------------------------------------------------------
% defaultAprilTagDetectorParams defines several hyperparameters that control the
% AprilTag detection algorithm. The detection algorithm is outlined below.
%  1. Apply gaussian blur to the input image.
%  2. Optionally, auto resize the input image if larger than 1000 pixels. (Default: opt out) 
%  3. Use custom adaptive thresholding to binarize the image.
%  4. Extract and filter contours to detect square shaped tag candidates.
%  5. Optionally, perform subpixel refinement of corner positions. (Default: opt in)
%  6. Remove perspective distortion and extract tag bits to analyze their
%     inner codification.
%  7. Perform error correction to identify valid tags.
%  8. Optionally, return intermediate outputs in steps 3-5 to debug. (Default: opt out)
%-------------------------------------------------------------------------------
function params = defaultAprilTagDetectorParams(tagFamily)

    % Step 1: GaussianSigma and GaussianWindowSize define the gaussian
    % filter used to blur the input image. GaussianWindowSize must be
    % always odd number. For images larger than 1000 pixels, increase the
    % GaussianWindowSize to the odd number after ceil(3*max(size(I))/1000).
    params.GaussianSigma = 0.0;
    params.GaussianWindowSize = 3;
    
    % Step 2: Enable automatic resizing of images larger than 1000 pixels.
    % Set this to true if there are detection failures of clearly visible
    % tags in images larger than 1000 pixels.
    params.AutoResizeLargeImage = false;
    
    % Step 3: Parameters for the custom adaptive thresholding to binarize
    % the image. Steps involved are:
    %  3.1: Get a max- and min-pool of the input image with a tile size
    %       specified by the DecimationFactor.
    %  3.2: Dilate the max-pool image, Imax, using a kernel of size DilationWindowSize.
    %  3.3: Erode the min-pool image, Imin, using a kernel of size ErosionWindowSize.
    %  3.4: Upsample the pooled images to original image size.
    %  3.5: Subtract the max-pool from min-pool image and apply the
    %       following thresholding logic to binarize the image:
    %  (diff < DetectorThreshold) ? 0 : (I > (Imin + diff / 2) ? 1 : 0)
    %  3.6: Dilate the final binarized image using EdgeDilationWindowSize.
    params.DecimationFactor = 1;
    params.DilationWindowSize = 3;
    params.ErosionWindowSize = 3;
    params.DetectorThreshold = 10;
    params.EdgeDilationWindowSize = 3;
    
    % Step 4: Filter contours to detect square shaped quad candidates.
    % QuadRatioThreshold compares the area of a quad to the area of its
    % convex hull and it must be between 0 and 1. The quad is then
    % approximated to hull at PolygonalPrecision.
    params.QuadRatioThreshold = 0.8;
    params.PolygonalPrecision = 8;
    
    % Step 5: Perform subpixel refinement of corner positions. 
    params.RefineEdges = true;
    params.RefinementWindowSize = 5;
    params.ZeroZoneWinSize = -1;
    params.MaxIterations = 40;
    params.ErrorTolerance = 0.001;
    
    % Step 6: Remove perspective distortion and extract tag bits to analyze
    % their inner codification. DecodeThresholdOffset is the avg brightness
    % offset that is used to threshold the final tag image and extract tag
    % bits.
    params.DecodeThresholdOffset = 20;
    params.NumBorderBits = 2;

    % Step 7: Perform error correction to identify valid tags.
    % Get tag family params if valid tag family is provided. These are
    % non-tunable hyper-parameters. Avoid changing these parameters.
    tagFamilyParams = getTagFamilyParams(tagFamily);
    params.MarkerSize = tagFamilyParams(1);
    params.MinimumDistance = tagFamilyParams(2);
    params.HammingThreshold = tagFamilyParams(3);

    % Step 8: Enable returning intermediate outputs in steps 3-5 to debug.
    params.Debug = false;
end