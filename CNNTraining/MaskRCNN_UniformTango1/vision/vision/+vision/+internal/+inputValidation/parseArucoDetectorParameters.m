function params = parseArucoDetectorParameters(options)
% parseArucoDetectorParameters Utility to validate NVPs of readArucoMarker
% and detectCharucoBoardPoints functions.

%   Copyright 2024 The MathWorks, Inc.
%#codegen

    arguments
        options.WindowSizeRange {mustBeNumeric, mustBeInteger, mustBePositive, iMustBeARange, mustBeGreaterThanOrEqual(options.WindowSizeRange,3)} = [3 23]
        options.WindowSizeStep (1,1) {mustBeNumeric, mustBeInteger, mustBePositive} = 10
        options.MarkerSizeRange {mustBeNumeric, mustBePositive, mustBeFinite, iMustBeARange, mustBeLessThanOrEqual(options.MarkerSizeRange,1)} = [0.0075 1]
        options.SquarenessTolerance (1,1) {mustBeNumeric, mustBePositive, mustBeFinite} = 0.03
        options.NumBorderBits (1,1) {mustBeNumeric, mustBeInteger, mustBePositive} = 1
        options.ResolutionPerBit (1,1) {mustBeNumeric, mustBeInteger, mustBePositive, mustBeLessThan(options.ResolutionPerBit,1000)} = 4 % Limit the max to 1000 to prevent out of memory issues. 
        options.RefineCorners {iMustBeLogicalScalar} = false
        options.RefinementWindowSize = []
        options.RefinementMaxIterations = []
        options.RefinementTolerance = []
    end
    
    % Validate corner refinement parameters.
    options.RefinementWindowSize = validateRefinementParam(...
        options.RefinementWindowSize, 5, options.RefineCorners, "RefinementWindowSize");
    options.RefinementMaxIterations = validateRefinementParam(...
        options.RefinementMaxIterations, 30, options.RefineCorners, "RefinementMaxIterations");
    options.RefinementTolerance = validateRefinementParam(...
        options.RefinementTolerance, 0.1, options.RefineCorners, "RefinementTolerance");
    
    params = defaultArucoDetectorParams();

    % Adaptive thresholding parameters
    params.AdaptiveThreshWinSizeMin = options.WindowSizeRange(1);
    params.AdaptiveThreshWinSizeMax = options.WindowSizeRange(2);
    params.AdaptiveThreshWinSizeStep = options.WindowSizeStep;

    % Contour filtering parameters
    params.MinMarkerPerimeterRate = 4*options.MarkerSizeRange(1);
    params.MaxMarkerPerimeterRate = 4*options.MarkerSizeRange(2);
    params.PolygonalApproxAccuracyRate = options.SquarenessTolerance;

    % Bit extraction parameters
    params.MarkerBorderBits = options.NumBorderBits;
    params.PerspectiveRemovePixelPerCell = options.ResolutionPerBit;
    
    % Subpixel corner refinement parameters
    params.CornerRefinementMethod = double(options.RefineCorners);
    params.CornerRefinementWinSize = options.RefinementWindowSize;
    params.CornerRefinementMaxIterations = options.RefinementMaxIterations;
    params.CornerRefinementMinAccuracy = options.RefinementTolerance;
end

%--------------------------------------------------------------------------
function iMustBeARange(range)
    
    validateattributes(range, {'numeric'}, {'size',[1 2]});

    coder.internal.errorIf(range(2) < range(1),'vision:aruco:invalidRangeVector');
end

%--------------------------------------------------------------------------
function iMustBeLogicalScalar(x)

    if ~isnumeric(x)
        validateattributes(x, {'logical'}, {'scalar'});
    else
        validateattributes(x, {'numeric'}, {'scalar','binary'});
    end
end

%--------------------------------------------------------------------------
function value = validateRefinementParam(input, defaultValue, doRefineCorners, paramName)
    if isempty(input)
        value = defaultValue;
    else
        coder.internal.errorIf(~doRefineCorners,'vision:aruco:cannotSpecifyRefinementParams', paramName);
        
        if paramName == "RefinementMaxIterations"
            validateattributes(input, {'numeric'}, {'scalar','positive','integer'});
        else
            validateattributes(input, {'numeric'}, {'scalar','positive','finite'});
        end

        value = input;
    end
end

%-------------------------------------------------------------------------------
% defaultArucoDetectorParams defines several hyperparameters that controls the
% ArUco marker detection algorithm. The detection algorithm is outlined below.
%  1. Adaptive thresholding to binarize the image.
%  2. Extract and filter contours to detect square shaped marker candidates.
%  3. Remove perspective distortion and extract marker bits to analyze their
%      inner codification.
%  4. Perform error correction to identify valid markers.
%  5. Optionally, perform subpixel refinement of corner positions.
%-------------------------------------------------------------------------------
function params = defaultArucoDetectorParams()
    % Hyperparameter values for ArUco detector.
    
    %% Step 1: Adaptive thresholding parameters.

    % The adaptiveThreshWinSizeMin and adaptiveThreshWinSizeMax parameters
    % represent the interval where the thresholding window sizes (in
    % pixels) are selected for the adaptive thresholding. The parameter
    % adaptiveThreshWinSizeStep indicates the increments of the window size
    % from adaptiveThreshWinSizeMin to adaptiveThreshWinSizeMax. For instance, 
    % for the default values there will be 2 thresholding steps with window 
    % sizes 3 and 23 (because the step size is 10). On each thresholding image, 
    % marker candidates will be extracted. Low values of window size can "break" 
    % the marker border if the marker size is  too large, causing it to not be 
    % detected. On the other hand, too large values can produce the same effect 
    % if the markers are too small, and can also reduce the performance.
    params.AdaptiveThreshWinSizeMin = 3;
    params.AdaptiveThreshWinSizeMax = 23;
    params.AdaptiveThreshWinSizeStep = 10;

    % The adaptiveThreshConstant parameter represents the constant value
    % added in the thresholding operation. Its default value is a good
    % option in most cases.
    params.AdaptiveThreshConstant = 7;

    %% Step 2: Contour filtering parameters.

    % minMarkerPerimeterRate and maxMarkerPerimeterRate determines the
    % minimum and maximum perimeter for marker contour to be detected. They
    % are defined as rates with respect to the maximum dimension of the input
    % image. That is, an image with size 640x480 and a minimum relative
    % marker perimeter of 0.05 will lead to a minimum marker perimeter of
    % 640x0.05 = 32 pixels. This also means that the minimum marker side in
    % pixels is 32/4 = 8 pixels.
    params.MinMarkerPerimeterRate = 0.03;
    params.MaxMarkerPerimeterRate = 4;

    % A polygonal approximation is applied to each candidate and only those
    % that approximate to a square shape are accepted. polygonalApproxAccuracyRate
    % determines the maximum error that the polygonal approximation can
    % produce. This parameter is relative to the candidate length (in pixels). 
    % So if the candidate has a perimeter of 100 pixels and the value of 
    % polygonalApproxAccuracyRate is 0.04, the maximum error would be 
    % 100x0.04 = 4 pixels. In most cases, the default value works fine, but 
    % higher error values could be necessary for highly distorted images.
    params.PolygonalApproxAccuracyRate = 0.03;

    % Minimum distance between any pair of corners in the same marker. It
    % is expressed relative to the marker perimeter. Minimum distance in
    % pixels is Perimeter * minCornerDistanceRate.
    params.MinCornerDistanceRate = 0.05;

    % Minimum distance between any pair of corners from two different
    % markers. It is expressed relative to the minimum marker perimeter of
    % the two markers. If two candidates are too close, the smaller one is
    % ignored.
    params.MinDistanceToBorder = 3;

    % Minimum distance to any of the marker corners to the image border (in
    % pixels). Markers partially occluded by the image border can be
    % correctly detected if the occlusion is small. However, if one of the
    % corners is occluded, the returned corner is usually placed in a wrong
    % position near the image border. If the position of marker corners is
    % important, for instance if you want to do pose estimation, it is
    % better to discard any markers whose corners are too close to the
    % image border. Elsewhere, it is not necessary.
    params.MinMarkerDistanceRate = 0.05;

    %% Step 3.1: Perspective distortion removal parameters

    % This parameter determines the number of pixels (per cell) in the
    % obtained image after correcting perspective distortion (including the
    % border). For instance, let’s assume we are dealing with markers of
    % 5x5 bits and border size of 1 bit. Then, the total number of cells/bits 
    % per dimension is 5 + 2*1 = 7 (the border has to be counted twice). The 
    % total number of cells is 7x7. If the value of perspectiveRemovePixelPerCell 
    % is 10, then the size of the obtained image will be 10*7 = 70 -> 70x70 pixels. 
    % A higher value of this parameter can improve the bits extraction process 
    % (up to some degree), however it can penalize the performance.
    params.PerspectiveRemovePixelPerCell = 4;

    % This parameter indicates the width of the margin of pixels on each
    % cell not considered for the determination of the cell bit. Represents
    % the rate respect to the total size of the cell, i.e.
    % perspectiveRemovePixelPerCell (default 0.13)
    params.PerspectiveRemoveIgnoredMarginPerCell = 0.13;
    
    %% Step 3.2: Bit extraction parameters

    % This parameter indicates the width of the marker border. It is
    % relative to the size of each bit. So, a value of 2 indicates the
    % border has the width of two internal bits. This parameter needs to
    % coincide with the border size of the markers user is using.
    params.MarkerBorderBits = 1;

    % This value determines the minimum standard deviation of the pixel
    % values to perform Otsu thresholding. If the deviation is low, it
    % probably means that all the square is black (or white) and applying
    % Otsu does not make sense. If this is the case, all the bits are set
    % to 0 (or 1) depending on whether the mean value is higher or lower
    % than 128.
    params.MinOtsuStdDev = 5.0;
    
    %% Step 4: Error correction parameters

    % The bits of the marker border should be black. This parameter
    % specifies the allowed number of erroneous bits in the border, i.e.
    % the maximum number of white bits in the border. It is represented
    % relative to the total number of bits in the marker.
    params.MaxErroneousBitsInBorderRate = 0.35;

    % Each marker family/dictionary has a theoretical maximum number of
    % bits that can be corrected (Dictionary.maxCorrectionBits). However,
    % this value can be modified by the errorCorrectionRate parameter. For
    % instance, if the allowed number of bits that can be corrected (for
    % the used dictionary) is 6 and the value of errorCorrectionRate is
    % 0.5, the real maximum number of bits that can be corrected is 6*0.5=3
    % bits. This value is useful to reduce the error correction
    % capabilities in order to avoid false positives.
    params.ErrorCorrectionRate = 0.6;

    %% Step 5: Corner refinement parameters (optional step)

    % 0 - Tag and corners detection based on the ArUco approach
    % 1 - ArUco approach and refine the corners locations using corner subpixel accuracy
    % 2 - ArUco approach and refine the corners locations using the contour-points line fitting
    % 3 - Tag and corners detection based on the AprilTag 2 approach
    params.CornerRefinementMethod = 0;
    
    % cornerRefinementWinSize parameter determines the window size of the
    % subpixel refinement process in pixels. High values can cause close
    % corners of the image to be included in the window area, so that the
    % corner of the marker moves to a different and incorrect location
    % during the process. Also, it can affect performance.
    params.CornerRefinementWinSize = 5;

    % These two parameters determine the stop criteria of the subpixel
    % refinement process. The cornerRefinementMaxIterations indicates the
    % maximum number of iterations and cornerRefinementMinAccuracy the
    % minimum error value before stopping the process. If the number of
    % iterations is too high, it may affect the performance. On the other
    % hand, if it is too low, it can result in poor subpixel refinement.
    params.CornerRefinementMaxIterations = 30;
    params.CornerRefinementMinAccuracy = 0.1;

    %% Global parameter: Used in all steps. 
    % To detect white markers, set detectInvertedMarker to true.
    params.DetectInvertedMarker = false;

    %% Aruco 3 functionality parameters
    % To enable the new and faster Aruco detection strategy, set
    % useAruco3Detection to true. Proposed in the paper: Romero-Ramirez et
    % al: Speeded up detection of squared fiducial markers (2018). Always
    % turn on subpixel corner refinement in case of Aruco3, due to upsampling.
    % That is, set cornerRefinementMethod to 1.
    params.UseAruco3Detection = false;

    % Minimum side length of a marker in the canonical image. Latter is the
    % binarized image in which contours are searched.
    params.MinSideLengthCanonicalImg = 32;

    % Minimum marker length w.r.t input image. It's range is [0,1], eq (2)
    % from paper. The parameter tau_i has a direct influence on the
    % processing speed.
    params.MinMarkerLengthRatioOriginalImg = 0.0;
end