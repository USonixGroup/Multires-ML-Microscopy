function I = generateCharucoBoard(varargin)

%   Copyright 2024 The MathWorks, Inc.

    narginchk(5,11)

    [imageSize, marginSize, charucoParams] = iValidateAndParseInputs(varargin{:});

    I = generateCharuco(imageSize, marginSize, charucoParams);
end

%--------------------------------------------------------------------------
function [imageSize, marginSize, params] = iValidateAndParseInputs(imageSize, patternDims, markerFamily, checkerSize, markerSize, options)
    arguments
        imageSize (:,:) {iValidateImageSize}
        patternDims (:,:) {iValidatePatternDims}
        markerFamily (1,1) string 
        checkerSize (1,1) {mustBeNumeric,mustBeInteger,mustBePositive}
        markerSize (1,1) {mustBeNumeric,mustBeInteger,mustBePositive}
        options.MarginSize (1,1) {mustBeNumeric,mustBeInteger,mustBePositive} = 10
        options.MinMarkerID (1,1) {mustBeNumeric,mustBeInteger,mustBeNonnegative} = 0
        options.OriginCheckerColor (1,1) string = "black"
    end
    
    % Validate marker family.
    family = iValidateMarkerFamily(markerFamily);

    % Validate checker size and marker size.
    iValidateCheckerSize(imageSize,patternDims,checkerSize);
    iValidateMarkerSize(checkerSize,markerSize);

    % Validate marker ids.
    params.IDs = iValidateMarkerIDs(patternDims, family, options.MinMarkerID);
    
    % Validate origin checker color.
    originCheckerColor = iValidateOriginCheckerColor(options.OriginCheckerColor);

    % Find board version based on origin checker color and number of rows.
    params.IsLegacyBoard = iFindBoardVersion(originCheckerColor, patternDims(1));
    
    % Set the number of border bits in the ArUco markers to 1.
    params.NumBorderBits = 1;

    % Populate other properties.
    params.PatternDims = patternDims;
    params.MarkerFamily = family;
    params.CheckerSize = checkerSize;
    params.MarkerSize = markerSize;
    
    marginSize = options.MarginSize;

    % Validate the input combination against the minimum required image area.
    iValidateMinImageSize(imageSize, marginSize, params);


end

%--------------------------------------------------------------------------
function iValidateImageSize(imageSize)

    validateattributes(imageSize, "numeric", {"finite", "integer", "positive", ...
        "numel", 2}, mfilename, "imageSize");
end

%--------------------------------------------------------------------------
function iValidatePatternDims(patternDims)
    
    validateattributes(patternDims, "numeric", {"finite", "integer", "positive", ...
        "numel", 2, '>=', 3}, mfilename, "patternDims");
end

%--------------------------------------------------------------------------
function validFamily = iValidateMarkerFamily(markerFamily)

    mustBeMember(markerFamily, vision.internal.supportedArucoMarkerFamilies);
    validFamily = char(markerFamily);
end

%--------------------------------------------------------------------------
function iValidateCheckerSize(imageSize, patternDims, checkerSize)
    
    vision.internal.errorIf(imageSize(1) < patternDims(1)*checkerSize, "vision:aruco:invalidImageHeight");
    vision.internal.errorIf(imageSize(2) < patternDims(2)*checkerSize, "vision:aruco:invalidImageWidth");
end

%--------------------------------------------------------------------------
function iValidateMarkerSize(checkerSize, markerSize)
    
    vision.internal.errorIf(checkerSize <= markerSize, "vision:aruco:invalidMarkerSize");
end

%--------------------------------------------------------------------------
function ids = iValidateMarkerIDs(patternDims, familyName, minMarkerID)
    
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
function validColor = iValidateOriginCheckerColor(color)

    validColor = validatestring(color, {'black','white'}, mfilename, "OriginCheckerColor");
end

%--------------------------------------------------------------------------
function isLegacyBoard = iFindBoardVersion(originCheckerColor, numRows)

    if originCheckerColor == "white" % numrows must be even
        vision.internal.errorIf(mod(numRows,2) ~= 0, "vision:aruco:numRowsMustBeEven");
        isLegacyBoard = true;
    else
        isLegacyBoard = false;
    end
end

%--------------------------------------------------------------------------
function iValidateMinImageSize(imageSize, marginSize, params)

    % Compute the number of bits along a dimension of the marker. Assuming
    % that each bit requires at least one pixel to represent, 
    %   markerSize = markerSizeInBits = markerSizeInPixels.
    markerSize = getMarkerSizeInBits(params.MarkerFamily, params.NumBorderBits);

    % Deduce the minimum checker size based on the min marker size.
    checkerToMarkerRatio = params.CheckerSize/params.MarkerSize;
    minCheckerSize = markerSize*checkerToMarkerRatio;

    % Compute the minimum board size using number of checkers in each
    % dimension.
    minBoardSize = minCheckerSize*max(params.PatternDims);

    % Min image size = board size + margin size on either side of the board.
    minImageSize = minBoardSize + 2*marginSize;

    vision.internal.errorIf(min(imageSize) < minImageSize, "vision:aruco:invalidImageSize", minImageSize);
end

%--------------------------------------------------------------------------
function markerSize = getMarkerSizeInBits(family, numBorderBits)

    if strcmpi(family, "DICT_ARUCO_ORIGINAL")
        numBitsPerSide = 5;
    else % DICT_nxn_FAMILYSIZE
        parts = strsplit(family, '_');
        numBitsPerSide = str2double(parts{2}(1)); % Extract the first digit of the second part
    end

    markerSize = numBitsPerSide + 2*numBorderBits;
end