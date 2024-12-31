function worldPoints = generateCheckerboardPoints(boardSize, squareSize)

% Copyright 2013-2024 MathWorks, Inc.

%#codegen

checkInputs(boardSize, squareSize);
worldPoints = vision.internal.calibration.patternWorldPointsImpl("checkerboard", boardSize, squareSize);


%--------------------------------------------------------------------------
function checkInputs(boardSize, squareSize)

validateattributes(boardSize, {'numeric'},...
    {'nonempty', 'vector', 'numel', 2, 'integer', 'positive', '>=', 3},....
    mfilename, 'boardSize'); %#ok<EMCA>

vision.internal.calibration.checkSquareSize(squareSize, mfilename);

if (~isempty(coder.target))    
    vision.internal.inputValidation.validateFixedSize(boardSize, 'boardSize');
    vision.internal.inputValidation.validateFixedSize(squareSize, 'squareSize');
end
