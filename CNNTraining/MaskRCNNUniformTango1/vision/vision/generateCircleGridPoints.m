function worldPoints = generateCircleGridPoints(patternDims, centerDistance, varargin)

% Copyright 2021-2024 The MathWorks, Inc.

    narginchk(2, 4)
    
    [patternName, patternDims, centerDistance] = ...
        validateAndParseInputs(patternDims, centerDistance, varargin{:});
    
    worldPoints = vision.internal.calibration.patternWorldPointsImpl(...
        patternName, patternDims, centerDistance);
    
end

%-------------------------------------------------------------------------------
function [patternName, patternDims, centerDistance] = validateAndParseInputs(patternDims, centerDistance, options)
    arguments
        patternDims
        centerDistance
        options.PatternType (1,1) string = "asymmetric"
    end    

    validateattributes(centerDistance, {'numeric'}, ...
        {'scalar', 'real', 'positive', 'finite', 'nonsparse'}, mfilename, 'centerDistance');    
    
    validateattributes(patternDims, {'numeric'},...
        {'nonempty', 'vector', 'numel', 2, 'real', 'integer', 'positive', '>=', 3}, ...
        mfilename, 'patternDims');

    options.PatternType = validatestring(options.PatternType, ...
                          {'symmetric','asymmetric'}, mfilename, 'PatternType');
    
    patternName = "circle-grid-" + options.PatternType;
    patternDims = double(patternDims);
    centerDistance = double(centerDistance);
end