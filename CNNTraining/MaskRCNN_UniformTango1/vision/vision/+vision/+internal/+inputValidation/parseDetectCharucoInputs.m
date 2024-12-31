function [detectorParams, intrinsics, showProgressBar, progressBarParent] = ...
    parseDetectCharucoInputs(isStereo, varargin)
% parseDetectCharucoInputs Utility to validate inputs of detectCharucoBoardPoints.

%   Copyright 2024 The MathWorks, Inc.

    idx = 2 + isStereo;
    mandatoryInputs = varargin(idx:idx+3);
    optionalInputs = varargin(idx+4:end);

    [patternDims, markerFamily, checkerSize, markerSize] = ...
        validateAndParseMandatoryInputs(mandatoryInputs{:});

    [detectorParams, intrinsics, showProgressBar, progressBarParent] = ...
        validateAndParseOptionalInputs(isStereo, patternDims, markerFamily, optionalInputs{:});

    detectorParams.PatternDims = patternDims;
    detectorParams.MarkerFamily = markerFamily;
    detectorParams.CheckerSize = checkerSize;
    detectorParams.MarkerSize = markerSize;
end

%-----------------------------------------------------------------------
function [patternDims, markerFamily, checkerSize, markerSize] = ...
    validateAndParseMandatoryInputs(patternDims, markerFamily, checkerSize, markerSize)
        
    patternDims = validatePatternDims(patternDims);
    markerFamily = validateMarkerFamily(markerFamily);
    checkerSize = validateMeasurement(checkerSize, 'checkerSize');
    markerSize = validateMeasurement(markerSize, 'markerSize');
    vision.internal.errorIf(checkerSize <= markerSize,"vision:aruco:invalidMarkerSize");
    
    %--------------------------------------------------------------------
    function patternDims = validatePatternDims(patternDims)
       validateattributes(patternDims, {'numeric'},...
            {'nonempty', 'vector', 'numel', 2, 'real', 'finite','integer', 'positive', '>=', 3}, ...
            'detectCharucoBoardPoints', 'patternDims');
    end

    %--------------------------------------------------------------------
    function value = validateMeasurement(value, varname)
       validateattributes(value, {'numeric'},...
            {'nonempty', 'scalar', 'real', 'finite', 'integer', 'positive'}, 'detectCharucoBoardPoints', varname);
    end
    
    %--------------------------------------------------------------------
    function markerFamily = validateMarkerFamily(markerFamily)
        validateattributes(markerFamily, {'char', 'string'}, {'nonempty', 'vector'}, 'detectCharucoBoardPoints', 'markerFamily');
        
        supportedFamilies = vision.internal.supportedCharucoBoardFamilies();
        markerFamily = validatestring(markerFamily, supportedFamilies, 'detectCharucoBoardPoints', 'markerFamily');
        markerFamily = char(markerFamily);
    end
end

%-----------------------------------------------------------------------
function [params, intrinsics, showProgressBar, progressBarParent] = ...
    validateAndParseOptionalInputs(isStereo, patternDims, markerFamily, camParams, options)
    arguments
        isStereo
        patternDims
        markerFamily
        camParams = cameraIntrinsics.empty
        options.ShowProgressBar (1,1) {mustBeA(options.ShowProgressBar, 'logical')} = false
        options.MinMarkerID (1,1) {mustBeNumeric,mustBeInteger,mustBeNonnegative} = 0
        options.OriginCheckerColor (1,1) string = "black"
        options.ProgressBarParent = []
        options.WindowSizeRange = [3 23]
        options.WindowSizeStep = 10
        options.MarkerSizeRange = [0.0075 1]
        options.SquarenessTolerance = 0.03
        options.NumBorderBits = 1
        options.ResolutionPerBit = 4
        options.RefineCorners = false
        options.RefinementWindowSize = []
        options.RefinementMaxIterations = []
        options.RefinementTolerance = []
    end

    % Parse ArUco parameters
    allNVPs = namedargs2cell(options);
    detectorNVPs = allNVPs(9:end);
    params = vision.internal.inputValidation.parseArucoDetectorParameters(detectorNVPs{:});
    
    % Populate default ChArUco detector parameters
    params = populateCharucoDetectorParameters(params);
    
    % Board properties
    validPatterns = {'black','white'};
    params.OriginCheckerColor = validatestring(options.OriginCheckerColor, ...
        validPatterns, 'detectCharucoBoardPoints', "OriginCheckerColor");

    params.IDs = validateAndParseMarkerIDs(patternDims, markerFamily, options.MinMarkerID);

    isNumRowsEven = mod(patternDims(1),2) == 0;
    if params.OriginCheckerColor == "white" % numrows must be even
        vision.internal.errorIf(~isNumRowsEven,"vision:aruco:numRowsMustBeEven");
        params.IsLegacyBoard = true;
    else
        params.IsLegacyBoard = false;
    end
    
    % Camera intrinsics
    intrinsics = validateAndParseIntrinsics(camParams, isStereo);

    % Waitbar properties.
    showProgressBar = options.ShowProgressBar;
    progressBarParent = options.ProgressBarParent;
end

%--------------------------------------------------------------------------
% Populate ChArUco detector parameters with their default values.
%--------------------------------------------------------------------------
function params = populateCharucoDetectorParameters(params)
    % Number of adjacent markers that must be detected to
    % return a charuco corner. Valid values: 1 or 2.
    params.MinMarkers = 2;

    % Try to use refine board to find markers that were not detected in the
    % ArUco detector function. First, based on the current detected marker
    % and the board layout, the function interpolates the position of the
    % missing markers. Then it tries to find correspondence between the
    % reprojected markers and the rejected candidates based on the
    % minRepDistance and errorCorrectionRate parameters. If camera
    % parameters and distortion coefficients are provided, missing markers
    % are reprojected using projectPoint function. If not, missing marker
    % projections are interpolated using global homography, and all the
    % marker corners in the board must have the same Z coordinate.
    params.TryRefineMarkers = false;

    %% The following parameters are needed if TryRefineMarkers is true. 

    % Minimum distance between the corners of the rejected candidate
    % and the reprojected marker in order to consider it as a correspondence.
    params.MinRepDistance = 10.0;

    % Rate of allowed erroneous bits with respect to the error correction
    % capability of the used dictionary. -1 ignores the error correction step.
    params.RefinementErrorCorrectionRate = 3.0;

    % Consider the four possible corner orders in the rejectedCorners array.
    % If it set to false, only the provided corner order is considered.
    params.CheckAllOrders = true;
end

%--------------------------------------------------------------------------
function ids = validateAndParseMarkerIDs(patternDims, familyName, minMarkerID)
    
    numMarkers = floor(patternDims(1)*patternDims(2)/2);
    ids = minMarkerID + (0:numMarkers-1);

    if strcmpi(familyName,"DICT_ARUCO_ORIGINAL")
        familySize = 1024;
    else
        familySize = 1000;
    end
    
    hasInvalidIDs = any(ids < 0 | ids >= familySize);
    vision.internal.errorIf(hasInvalidIDs,"vision:aruco:invalidMinMarkerID",...
        familyName, familySize-1);
end

%--------------------------------------------------------------------------
function intrinsics = validateAndParseIntrinsics(camParams, isStereo)

    if isStereo
        if ~isequal(camParams, cameraIntrinsics.empty)
            if isa(camParams,'stereoParameters') && isscalar(camParams)
                intrinsics1 = opencvIntrinsics(camParams.CameraParameters1);
                intrinsics2 = opencvIntrinsics(camParams.CameraParameters2);
            elseif isa(camParams,'cameraIntrinsics') && numel(camParams)==2
                intrinsics1 = opencvIntrinsics(camParams(1));
                intrinsics2 = opencvIntrinsics(camParams(2));
            else
                error(message("vision:calibrate:invalidCameraIntrinsics"))
            end
        else
            intrinsics1 = opencvIntrinsics(camParams);
            intrinsics2 = opencvIntrinsics(camParams);
        end
        intrinsics = [intrinsics1, intrinsics2];
    else
        if ~isequal(camParams, cameraIntrinsics.empty)
            validTypes = {'cameraIntrinsics','cameraParameters'};
            validateattributes(camParams, validTypes, {'nonempty','scalar'}, ...
                'detectCharucoBoardPoints', 'intrinsics');
        end
        intrinsics = opencvIntrinsics(camParams);
    end
end

%--------------------------------------------------------------------------
function params = opencvIntrinsics(cameraParams)
    params.UseIntrinsics = ~isempty(cameraParams);
    if params.UseIntrinsics
        if isa(cameraParams, 'cameraParameters')
            intrinsics = cameraParams.Intrinsics;
        else
            intrinsics = cameraParams;
        end

        % Convert CVT intrinsics to OpenCV.
        [Kcolumnmajor, D] = cameraIntrinsicsToOpenCV(intrinsics);
    
        % Convert K to row major layout to pass it to OpenCV directly.
        K = Kcolumnmajor';
    else
        K = eye(3);
        D = zeros(1,8);
    end
    params.CameraMatrix = K;
    params.DistCoeffs = D;
end