function [detectorParams, showProgressBar, progressBarParent] = ...
    parseDetectAprilGridInputs(isStereo, varargin)
% parseDetectAprilGridInputs Utility to validate inputs of detectAprilGridPoints.

%   Copyright 2024 The MathWorks, Inc.

    idx = 2 + isStereo;
    mandatoryInputs = varargin(idx:idx+1);
    optionalInputs = varargin(idx+2:end);

    [patternDims, tagFamily] = validateAndParseMandatoryInputs(mandatoryInputs{:});

    [detectorParams, showProgressBar, progressBarParent] = ...
        validateAndParseOptionalInputs(patternDims, tagFamily, optionalInputs{:});

    detectorParams.PatternDims = patternDims;
    detectorParams.TagFamily = tagFamily;
end

%-----------------------------------------------------------------------
function [patternDims, tagFamily] = validateAndParseMandatoryInputs(patternDims, tagFamily)
        
    patternDims = validatePatternDims(patternDims);
    tagFamily = validateTagFamily(tagFamily);
    
    %--------------------------------------------------------------------
    function patternDims = validatePatternDims(patternDims)
       validateattributes(patternDims, {'numeric'},...
            {'nonempty', 'vector', 'numel', 2, 'real', 'finite','integer', 'positive', '>=', 2}, ...
            'detectAprilGridPoints', 'patternDims');
    end
    
    %--------------------------------------------------------------------
    function tagFamily = validateTagFamily(tagFamily)
        validateattributes(tagFamily, {'char', 'string'}, {'nonempty', 'vector'},...
            'detectAprilGridPoints', 'tagFamily');
        
        supportedFamilies = vision.internal.supportedAprilGridFamilies();
        tagFamily = validatestring(tagFamily, supportedFamilies,...
            'detectAprilGridPoints', 'tagFamily');
        tagFamily = char(tagFamily);
    end
end

%-----------------------------------------------------------------------
function [params, showProgressBar, progressBarParent] = ...
    validateAndParseOptionalInputs(patternDims, tagFamily, options)
    arguments
        patternDims
        tagFamily
        options.ShowProgressBar (1,1) {mustBeA(options.ShowProgressBar, 'logical')} = false
        options.MinTagID (1,1) {mustBeNumeric,mustBeInteger,mustBeNonnegative} = 0
        options.DecimationFactor (1,1) {mustBeNumeric, mustBeFinite, mustBeNonsparse, ...
            mustBePositive, mustBeGreaterThanOrEqual(options.DecimationFactor,1)} = 1
        options.GaussianSigma (1,1) {mustBeNumeric, mustBeFinite, mustBeNonsparse, ...
            mustBeNonnegative} = 0
        options.NumBorderBits (1,1) {mustBeNumeric, mustBeInteger, mustBePositive} = 2
        options.ProgressBarParent = []
    end

    [params.IDs, params.MinTagID, params.MaxTagID] = validateAndParseTagIDs(...
        patternDims, tagFamily, options.MinTagID);

    params.GaussianSigma = options.GaussianSigma;
    params.DecimationFactor = options.DecimationFactor;
    params.NumBorderBits = options.NumBorderBits;

    % Waitbar properties.
    showProgressBar = options.ShowProgressBar;
    progressBarParent = options.ProgressBarParent;
end

%--------------------------------------------------------------------------
function [ids, minTagID, maxTagID] = validateAndParseTagIDs(patternDims, familyName, minTagID)
    
    numTags = prod(patternDims);
    ids = minTagID + (0:numTags-1);
    maxTagID = minTagID + numTags-1;
    if strcmpi(familyName, "tag36h11")
        familySize = 587;
    elseif strcmpi(familyName, "tag25h9")
        familySize = 35;
    else
        familySize = 30;
    end
    
    hasInvalidIDs = any(ids < 0 | ids >= familySize);
    vision.internal.errorIf(hasInvalidIDs,"vision:apriltag:invalidMinTagID",...
        familySize-1, familyName);
end