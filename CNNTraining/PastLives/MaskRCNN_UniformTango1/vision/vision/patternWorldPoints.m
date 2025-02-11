function worldPoints = patternWorldPoints(patternName, patternDims, pointsDistance, tagSpacing)

% Copyright 2024 The MathWorks, Inc.

%#codegen

    arguments
        patternName    (1,1) string
        patternDims    (:,:) {mustBeNonempty,mustBeNumeric,mustBeFinite,mustBeInteger,mustBePositive}
        pointsDistance (1,1) {mustBeNumeric,mustBeReal,mustBeFinite,mustBePositive,mustBeNonsparse}
        tagSpacing = []
    end

    % Validate patternName.
    patternName = validatePatternName(patternName);

    % Additional validations for patternDims.
    validatePatternDims(patternDims, patternName);

    % Validate syntax.
    if strcmp(patternName, "aprilgrid")
        % patternWorldPoints("aprilgrid",patternDims,pointsDistance,tagSpacing)
        narginchk(4,4);

        % Validate tagSpacing.
        validateTagSpacing(tagSpacing);
    else
        % patternWorldPoints(patternName,patternDims,pointsDistance)
        narginchk(3,3);
    end
    
    % Compute world points.
    worldPoints = vision.internal.calibration.patternWorldPointsImpl(...
        patternName, patternDims, pointsDistance, tagSpacing);
end

%--------------------------------------------------------------------------
function patternName = validatePatternName(patternName)

    validPatterns = {'checkerboard', 'charuco-board', 'aprilgrid', ...
        'circle-grid-symmetric', 'circle-grid-asymmetric'};
    patternName = validatestring(patternName, validPatterns, mfilename, "patternName");
end

%--------------------------------------------------------------------------
function validatePatternDims(patternDims, patternName)

    if contains(patternName, "grid")
        minSize = 2;
    else
        minSize = 3;
    end
    validateattributes(patternDims, "numeric", {"numel", 2, '>=', minSize}, ...
        mfilename, "patternDims");
end

%--------------------------------------------------------------------------
function validateTagSpacing(tagSpacing)

    validateattributes(tagSpacing, "numeric", {"nonempty", "scalar", "real", "finite", "positive", "nonsparse"}, ...
        mfilename, "tagSpacing");
end