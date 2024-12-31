function RGB = insertObjectAnnotation(I, shape, position, labelTemp, varargin)

% Copyright 2018-2024 The MathWorks, Inc.

%#codegen
%#ok<*EMCA>

coder.allowpcode('plain');
if isSimMode()
    label = labelTemp;
    if isstring(label) || iscategorical(label)
        label = manageMissingLabels(label);
        label = convertStringsToChars(string(label));
    end
else
    % Convert string and categorical types to cell array type
    % for code generation.
    if isstring(labelTemp) || iscategorical(labelTemp)
        label = cellstr(labelTemp);
    else
        label = labelTemp;
    end
end
%% == Parse inputs and validate ==
[RGB, shape, position, label, lineWidth, showOrientation, color, ...
    textColor, textBoxOpacity, fontSize, font, isEmpty] = ...
    validateAndParseInputs(I, shape, position, label, varargin{:});

% handle empty I or empty position
if isEmpty
    return;
end


if isSimMode()
    RGB = insertShape(RGB, shape, position, ...
        'LineWidth', lineWidth, ...
        'Color',     color);
else
    RGB(:) = insertShape(RGB, shape, position, ...
        'LineWidth', lineWidth, ...
        'Color',     color);
end

if showOrientation
    RGB = createOrientationAnnotation(RGB, position, lineWidth, color);   
end

% If the shape and position define a rotated rectangle or ellipse, the
% annotation text will be centered in the shape.
isCenteredText = (strcmp(shape,"rectangle") && size(position,2) == 5) ...
    || strcmp(shape,"ellipse");

[textLocAndWidth] = getTextLocAndWidth(shape, position, lineWidth);
textPosition    = textLocAndWidth(:,1:2);
shapeWidth  = textLocAndWidth(:,3);
shapeHeight = textLocAndWidth(:,4);
if isCenteredText
    RGB = insertText(RGB, textPosition, label, ...
        'FontSize',     fontSize, ...
        'Font',         font, ...
        'FontColor',    textColor, ...
        'TextBoxColor', color, ...
        'BoxOpacity',   textBoxOpacity, ...
        'AnchorPoint',  'Center', ...
        'ShapeWidth',   shapeWidth, ...
        'ShapeHeight',  shapeHeight);
else
    RGB = insertText(RGB, textPosition, label, ...
        'FontSize',     fontSize, ...
        'Font',         font, ...
        'FontColor',    textColor, ...
        'TextBoxColor', color, ...
        'BoxOpacity',   textBoxOpacity, ...
        'AnchorPoint',  'LeftBottom', ...
        'ShapeWidth',   shapeWidth, ...
        'ShapeHeight',  shapeHeight);
end

%==========================================================================
% Parse inputs and validate
%==========================================================================
function [RGB, shape,position,outLabel,lineWidth,showOrientation,color,...
    textColor,textBoxOpacity,fontSize,font,isEmpty] = ...
    validateAndParseInputs(I, shape, position, label, varargin)

%--input image--
checkImage(I);
RGB = convert2RGB(I);
inpClass = class(I);

%--shape--
shape = validatestring(shape,...
    {'rectangle','circle','projected-cuboid','ellipse'}, ...
    mfilename,'SHAPE', 3);

%--position--
% position data type does not depend on input data type
vision.internal.inputValidation.validateNotObject(position, 'vision', 'position');
validateattributes(position, {'numeric'}, ...
    {'real','nonsparse', 'finite'}, mfilename,'POSITION', 3);

% Compute number of shapes.
numShapes = numberOfShapes(position);

%--isEmpty--
isEmpty = isempty(I) || isempty(position);

%--label--
checkLabel(label);

if ischar(label)
    numLabels = 1;
else
    numLabels = length(label);
end
outLabel  = label;

%--other optional parameters--
if isSimMode()
    [lineWidth, showOrientation, color, textColor, textBoxOpacity, fontSize, font] = ...
        validateAndParseOptInputs_sim(inpClass,shape,position,varargin{:});
else
    [lineWidth, showOrientation, color, textColor, textBoxOpacity, fontSize, font] = ...
        validateAndParseOptInputsCodegen(inpClass,shape,position,varargin{:});
end
crossCheckInputs(shape, position, numLabels, color, textColor);
color = getColorMatrix(inpClass, numShapes, color);
textColor = getColorMatrix(inpClass, numShapes, textColor);

%==========================================================================
function flag = isSimMode()

flag = isempty(coder.target);

%==========================================================================
function [lineWidth, showOrientation, color, textColor, textBoxOpacity, fontSize, font] = ...
    validateAndParseOptInputs_sim(inpClass,shape,position,varargin)
% Validate and parse optional inputs

defaults = getDefaultParameters(inpClass);
% Setup parser
parser = inputParser;
parser.CaseSensitive = false;
parser.FunctionName  = mfilename;

parser.addParameter('LineWidth', defaults.LineWidth);
parser.addParameter('ShowOrientation', defaults.ShowOrientation);
parser.addParameter('Color', defaults.Color);         % old parameter
parser.addParameter('AnnotationColor', defaults.Color);
parser.addParameter('TextColor', defaults.TextColor); % old parameter
parser.addParameter('FontColor', defaults.TextColor);
parser.addParameter('TextBoxOpacity', defaults.TextBoxOpacity, ...
    @checkTextBoxOpacity);
parser.addParameter('FontSize', defaults.FontSize, @checkFontSize);
parser.addParameter('Font', defaults.Font);

%Parse input
parser.parse(varargin{:});

unsetValues = parser.UsingDefaults;
IsShowOrientationSet = ~ismember('ShowOrientation', unsetValues);

lineWidth       = checkLineWidth(parser.Results.LineWidth);
showOrientation = checkShowOrientation(parser.Results.ShowOrientation, shape, position, IsShowOrientationSet);
color           = vision.internal.parseInserterColor("Color", "AnnotationColor", parser, mfilename, inpClass);
textColor       = vision.internal.parseInserterColor("TextColor", "FontColor", parser, mfilename, inpClass);
textBoxOpacity  = double(parser.Results.TextBoxOpacity);
fontSize        = double(parser.Results.FontSize);
font            = vision.internal.checkFont(parser.Results.Font, mfilename);
%==========================================================================
function [lineWidth, showOrientation, color, textColor, textBoxOpacity, fontSize, font] = ...
    validateAndParseOptInputsCodegen(inpClass,shape,position,varargin)
% Validate and parse optional inputs

defaultsNoVal = getDefaultParametersNoVal();
defaults      = getDefaultParameters(inpClass);
properties    = getEmlParserProperties();

optarg = eml_parse_parameter_inputs(defaultsNoVal,properties,varargin{:});

lineWidth1 = (eml_get_parameter_value(optarg.LineWidth, ...
    defaults.LineWidth, varargin{:}));
showOrientationIn = (eml_get_parameter_value(optarg.ShowOrientation, ...
    defaults.ShowOrientation, varargin{:}));
color = vision.internal.codegen.parseInserterColor('Color', 'AnnotationColor', ...
    optarg, coder.const(mfilename), inpClass, defaults, varargin{:});
textColor = vision.internal.codegen.parseInserterColor('TextColor', 'FontColor', ...
    optarg, coder.const(mfilename), inpClass, defaults, varargin{:});
textBoxOpacity = (eml_get_parameter_value(optarg.TextBoxOpacity, ...
    defaults.TextBoxOpacity, varargin{:}));
fontSize = (eml_get_parameter_value(optarg.FontSize, ...
    defaults.FontSize, varargin{:}));
font = (eml_get_parameter_value(optarg.Font, ...
    defaults.Font, varargin{:}));

lineWidth1     = checkLineWidth(lineWidth1);
lineWidth      = double(lineWidth1);

IsShowOrientationSet = logical(optarg.ShowOrientation);

showOrientationIn = checkShowOrientation(showOrientationIn,shape,position,IsShowOrientationSet);
showOrientation = logical(showOrientationIn);

checkTextBoxOpacity(textBoxOpacity);
textBoxOpacity = double(textBoxOpacity);

checkFontSize(fontSize);
fontSize    = int32(fontSize);% const cast is done in insertText

coder.extrinsic('vision.internal.checkFont');
font        = coder.const(vision.internal.checkFont(font, mfilename)); % readjusted case

%==========================================================================
function checkImage(I)
% Validate input image

% No objects allowed.
vision.internal.inputValidation.validateNotObject(I, 'vision', 'I');

validateattributes(I,{'uint8', 'uint16', 'int16', 'double', 'single'}, ...
    {'real','nonsparse'}, mfilename, 'I', 1)
% input image must be 2d or 3d (with 3 planes)
errIf0((ndims(I) > 3) || ((size(I,3) ~= 1) && (size(I,3) ~= 3)), ...
    'vision:dims:imageNot2DorRGB');

coder.internal.assert(coder.internal.isConst(size(I,3)), ...
    'vision:insertObjectAnnotation:image3rdDimFixed');
%==========================================================================
function checkLabel(label)
% Validate label

if isnumeric(label)
    vision.internal.inputValidation.validateNotObject(label, 'vision', 'label');
    if ~isempty(label)
        validateattributes(label, {'numeric'}, ...
            {'real', 'nonsparse', 'nonnan', 'finite', 'vector'}, ...
            mfilename, 'LABEL');
    end
else
    if ischar(label)
        % allow empty string ('')
        validateattributes(label,{'char'}, {}, ...
            mfilename, 'LABEL');
        labelCell = {label};
    else
        % allow empty cell {} or 0x1 cell to enable workflows where
        % position is empty. The crossCheckInputs function validates
        % whether the number of labels matches the number of
        % positions/objects.
        if isempty(label)
            validateattributes(label,{'cell'}, {},...
                mfilename, 'LABEL');
        else
            validateattributes(label,{'cell'}, {'vector'}, ...
                mfilename, 'LABEL');
        end
        allLabelCellsChar = true;
        for i=1:length(label)
            vision.internal.inputValidation.validateNotObject(label{i}, 'vision', 'label');
            allLabelCellsChar = allLabelCellsChar && ischar(label{i});
        end
        errIf0(~allLabelCellsChar, ...
            'vision:insertObjectAnnotation:labelCellNonChar');
        labelCell = label;
    end
    
    % manually generate hasNewLine and hasCarriageReturn.
    % sprintf('\n')==10; sprintf('\r')==13
    % 'my\nname' is fine; sprintf('my\nname') is not accepted
    hasNewLine     = false;
    hasCarriageRet = false;
    for i = 1:length(labelCell)
        for j = 1:length(labelCell{i})
            hasNewLine     = hasNewLine || labelCell{i}(j)==10;
            hasCarriageRet = hasCarriageRet || labelCell{i}(j)==13;
        end
    end
    
    errIf0(hasNewLine || hasCarriageRet, ...
        'vision:insertObjectAnnotation:labelNewLineCR');
end

%==========================================================================
function crossCheckInputs(shape, position, numLabels, color, textColor)
% Cross validate inputs
if ndims(position)==3
    [~, numColsPositions,numRowsPositions] = size(position);
else
    [numRowsPositions, numColsPositions] = size(position);
end

numPtsForShape = getNumPointsForShape(shape,position);
numShapeColors = getNumColors(color);
numTextColors  = getNumColors(textColor);

% cross check shape and position (cols)
% size of position: for rectangle Mx4 or Mx5, for circle Mx3
if isempty(position)
    numRowsPositions = 0;
else
    errIf0(numPtsForShape ~= numColsPositions, ...
        'vision:insertObjectAnnotation:invalidNumColPos');
end

if strcmp(shape,'ellipse')
    % Major axis length must be >= minor axis length.
    errIf0(any(position(:,3) < position(:,4)),...
        'vision:insertShape:invalidEllipseMajorMinorAxisLength');
end

% cross check label and position (rows)
errIf0((numLabels ~=1) && (numLabels ~= numRowsPositions), ...
    'vision:insertObjectAnnotation:invalidNumLabels');

% cross check color and position (rows). Empty color is caught here
errIf0((numShapeColors ~= 1) && (numRowsPositions ~= numShapeColors), ...
    'vision:insertObjectAnnotation:invalidNumPosNumColor');

% cross check text color and position (rows). Empty color is caught here
errIf0((numTextColors ~= 1) && (numRowsPositions ~= numTextColors), ...
    'vision:insertObjectAnnotation:invalidNumPosNumColor');

%==========================================================================
function colorOut = getColorMatrix(inpClass, numShapes, color)

color = colorRGBValue(color, inpClass);
if (size(color, 1)==1)
    colorOut = repmat(color, [numShapes 1]);
else
    colorOut = color;
end

%==========================================================================
function numPts = getNumPointsForShape(shape, position)

switch shape
    case 'rectangle'
        if size(position,2) ~= 5 % check if rectangle is axis-aligned
            numPts = 4;% rectangle: [x y width height]
        else
            numPts = 5;% rotated rectangle: [xctr yctr width height yaw]
        end
    case 'circle'
        numPts = 3;% circle: [x y radius]
    case 'projected-cuboid' % shape is projected cuboid  
        if ndims(position)==3 % vertices representation
            numPts = 2;
        elseif ndims(position)==2 && all(size(position,[1,2])==[8 2])
            numPts = 2;
        else % rectangle representation
            numPts = 8;
        end
    case 'ellipse'
        numPts = 5;% ellipse: [xCenter yCenter major minor yaw]
    otherwise
        % code flow should not reach here.
        assert(false,'Incorrect shape.')
        numPts = 0;
end

%==========================================================================
function numColors = getNumColors(color)

% Get number of colors
numColors = 1;
if isnumeric(color)
    if isempty(color)
        numColors = 0;
    else
        numColors = size(color,1);
    end
elseif iscell(color) % if color='red', it is converted to cell earlier
    numColors = length(color);
end

%==========================================================================
function defaultFont = getDefaultFont_sim()

persistent origDefFont

if isempty(origDefFont)
    origDefFont = vision.internal.getDefaultFont();
end
defaultFont = origDefFont;

%==========================================================================
function defaultFont = getDefaultFont_cg()

coder.extrinsic('vision.internal.getDefaultFont');
defaultFont = coder.internal.const(vision.internal.getDefaultFont());


%==========================================================================
function defaults = getDefaultParameters(inpClass)

% Get default values for optional parameters
% default color 'black', default text color 'yellow'
black = [0 0 0];
switch inpClass
    case {'double', 'single'}
        yellow = [1 1 0];
    case 'uint8'
        yellow = [255 255 0];
    case 'uint16'
        yellow = [65535  65535  0];
    case 'int16'
        yellow = [32767  32767 -32768];
        black = [-32768  -32768  -32768];
end

if isSimMode()
    origDefFont = getDefaultFont_sim();
else
    origDefFont = getDefaultFont_cg();
end

defaults = struct(...
    'LineWidth', 1, ...
    'ShowOrientation', false, ...
    'Color', yellow, ...
    'AnnotationColor', yellow, ...
    'TextColor',  black, ...
    'FontColor',  black, ...
    'TextBoxOpacity', 0.6,...
    'FontSize', 12,...
    'Font', origDefFont);

%==========================================================================
function defaults = getDefaultParametersNoVal()

defaults = struct(...
    'LineWidth', uint32(0), ...
    'ShowOrientation', uint32(0), ...
    'Color',  uint32(0), ...
    'AnnotationColor',  uint32(0), ...
    'TextColor', uint32(0), ...
    'FontColor', uint32(0), ...
    'TextBoxOpacity', uint32(0),...
    'FontSize', uint32(0),...
    'Font', uint32(0));

%==========================================================================
function properties = getEmlParserProperties()

properties = struct( ...
    'CaseSensitivity', false, ...
    'StructExpand',    true, ...
    'PartialMatching', false);

%==========================================================================
function errIf0(condition, msgID)

coder.internal.errorIf(condition, msgID);

%==========================================================================
function lineWidthOut = checkLineWidth(lineWidth)
% Validate 'LineWidth'

validateattributes(lineWidth, {'numeric'}, ...
    {'nonsparse', 'integer', 'scalar', 'real', 'positive'}, ...
    'insertObjectAnnotation', 'LineWidth');

lineWidthOut = lineWidth;

%==========================================================================
function showOrientationOut = checkShowOrientation(showOrientation, shape, position, IsShowOrientationSet)
% Validate 'ShowOrientation'
vision.internal.inputValidation.validateLogical(showOrientation,"ShowOrientation")

% Rotated rectangle and ellipse can show orientation arrow.
canShowOrientation = (strcmp(shape,'rectangle') && size(position,2) == 5) || ...
    strcmp(shape,'ellipse');

% Error if the shape is not ellipse or rotated rectangle.
errIf0(IsShowOrientationSet && ~canShowOrientation, ...
    'vision:insertObjectAnnotation:invalidShowOrientationUse');
if canShowOrientation
    showOrientationOut = showOrientation;
else
    showOrientationOut = false;
end

%==========================================================================
function tf = checkTextBoxOpacity(opacity)
% Validate 'TextBoxOpacity'
vision.internal.inputValidation.validateNotObject(opacity,'vision','opacity');
validateattributes(opacity, {'numeric'}, {'nonempty', 'nonnan', ...
    'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
    mfilename, 'TextBoxOpacity');
tf = true;

%==========================================================================
function tf = checkFontSize(FontSize)
% Validate 'FontSize'
vision.internal.inputValidation.validateNotObject(FontSize,'vision','FontSize');
validateattributes(FontSize, {'numeric'}, ...
    {'nonempty', 'integer', 'nonsparse', 'scalar', '>=', 8, '<=', 72}, ...
    mfilename, 'FontSize');
tf = true;

%==========================================================================
function textLocAndWidth = getTextLocAndWidth(shape, position, lineWidth)
% This function computes the text location and the width of the shape
% Text location:
%   * It is the bottom-left corner (x,y) of the label text box, except for
%     rotated rectangles. Rotated rectangles have the text box location as
%     centered.
%   * Label text box is left aligned with shape
%   * Since label text box is placed above the shape (i.e., bottom border
%     of the label text box touches the top-most point of the shape),
%     (x, y) is computed as follows:
%     For 'rectangle' shape, when axis-aligned, (x, y) is the top-left
%     corner of the shape. When rotated, (x, y) is at the center of the
%     shape.
%     For 'circle' shape, (x, y) is the top-left corner of the rectangle
%     that encloses the shape (circle)
%     For 'projected-cuboid' shape, (x,y) is the top left vertex of the
%     front face in rectangles representation and (x,y) is the left most
%     vertex in the vertices representation.
%     For 'ellipse' shape, (x, y) is the top-left corner of the rectangle
%     that encloses the shape (ellipse)
% Width of label text box:
%   * For 'rectangle' shape, when axis-aligned, Width of label text
%     box = width of rectangle. When rotated, box is fit to text length.
%     This means that the box will expand as needed based on the number of
%     characters in the label.
%   * For 'circle' shape, Width of label text box = diameter of circle
%   * For 'ellipse' shape, Width of label text box is fit to text length.


halfLineWidth = floor(lineWidth/2);
lineWidthAdj = 2*halfLineWidth; % adjusted line width
position = int32(position);
switch shape
    case 'rectangle'
        % position must not be a column vector
        % [x y width]
        textLocAndWidth = position(:,1:4);
        if size(position,2) == 5
            % If the shape is a rotated rectangle, the label text is
            % centered and the text box is dynamically in length based on
            % the number of charaters in the label.
            textLocAndWidth(:,3) = zeros(size(textLocAndWidth,1), 1, 'like', textLocAndWidth);
        else
            textLocAndWidth(:,2) = textLocAndWidth(:,2) - int32(1) - int32(halfLineWidth);
            textLocAndWidth(:,1) = textLocAndWidth(:,1) - int32(halfLineWidth);
            textLocAndWidth(:,3) = textLocAndWidth(:,3) + int32(lineWidthAdj);
        end
    case 'circle'
        % [x y width] = [center_x-radius center_y-radius-1 2*radius+1]
        textLocAndWidth = [position(:,1)-position(:,3) - int32(halfLineWidth)...
            position(:,2)-position(:,3) - int32(1) - int32(halfLineWidth) ...
            2*position(:,3) + int32(lineWidthAdj+1), ...
            2*position(:,3) + int32(lineWidthAdj+1)];
    case 'projected-cuboid' 
        % projected cuboids can be [8,2,M] or [M,8] depending on vertices
        % or rectangles representation.
        if ndims(position)==3
            % For vertices representation place the text at the left most
            % vertex.
            textLocAndWidth = zeros(size(position,3),4);         
            [textLocAndWidth(:,1),loc] = (min(position(1:8,1,:)));
            loc = squeeze(loc);
            for i=1:size(position,3)
                textLocAndWidth(i,2) = squeeze(position(loc(i),2,i));
            end
            textLocAndWidth(:,4) = 0;
            textLocAndWidth(:,3) = 0;
        elseif all(size(position)==[8 2])
            % This block handles the case when single cuboid vertices are
            % provided. MATLAB squeezes last singleton: [8,2,1] to [8,2].
            textLocAndWidth = zeros(1,4);
            [textLocAndWidth(:,1),loc] = (min(position(1:8,1,:)));
            loc = squeeze(loc);
            for i=1:size(position,3)
                textLocAndWidth(i,2) = squeeze(position(loc(i),2,i));
            end
            textLocAndWidth(:,4) = 0;
            textLocAndWidth(:,3) = 0;
        else
            % For rectangle representation place the text at the top left
            % of front face.
            textLocAndWidth = zeros(size(position,1),4);
            textLocAndWidth = position(:,1:4);
            textLocAndWidth(:,4) = 0;
            textLocAndWidth(:,3) = 0;
        end
    case 'ellipse'
        % For ellipse, position is an M-by-5 matrix of 
        % [xCenter yCenter major minor yaw].
        % The label text is located at the center of ellipse and the width
        % of the text box is fit to text length.
        textLocAndWidth = position(:,1:4);
        textLocAndWidth(:,3) = zeros(size(textLocAndWidth,1), 1, 'like', textLocAndWidth);
    otherwise
        % Code flow should not reach here.
        assert(false,"Invalid shape choice.")
end

%==========================================================================
function rgb = createOrientationAnnotation(rgb, position, lineWidth, color)
% Insert an orientation arrow annotation that denotes the "head" of a
% rotated rectangle.
vertices = vision.internal.bbox.rotatedRectangleHeadDisplay(rgb, position, lineWidth);

% Use the vertices of the 'head' of the rotated rectangle to create a line
% of length
dtClass = coder.internal.const(class(rgb));
shapeFill = coder.internal.const(true);
smoothEdges = coder.internal.const(true);

% Insert heading arrow
if isSimMode()
    rgb = visionInsertShape(int32(3), shapeFill, 1, 1, smoothEdges, dtClass, rgb, int32(vertices), color);

else
    rgb = vision.internal.buildable.insertShapeBuildable.insertShape(int32(3), ...
        shapeFill, 1, 1, smoothEdges, dtClass, rgb, int32(vertices), color);
end

%==========================================================================
function inRGB = convert2RGB(I)

if ismatrix(I)
    inRGB = repmat(I,[1 1 3]);
else
    inRGB = I;
end

%==========================================================================
function outColor = colorRGBValue(inColor, inpClass)

if isnumeric(inColor)
    outColor = cast(inColor, inpClass);
else
    if iscell(inColor)
        textColorCell = inColor;
    else
        textColorCell = {inColor};
    end
    
    numColors = length(textColorCell);
    outColor = zeros(numColors, 3, inpClass);
    
    for ii=1:numColors
        supportedColorStr = {'blue','green','red','cyan','magenta','yellow',...
            'black','white','b','k'};
        % https://www.mathworks.com/help/techdoc/ref/colorspec.html
        colorValuesFloat = [0 0 1;0 1 0;1 0 0;0 1 1;1 0 1;1 1 0;0 0 0;1 1 1;0 0 1;0 0 0];
        idx = strcmp(textColorCell{ii}, supportedColorStr);
        switch inpClass
            case {'double', 'single'}
                outColor(ii, :) = colorValuesFloat(idx, :);
            case {'uint8', 'uint16'}
                colorValuesUint = colorValuesFloat*double(intmax(inpClass));
                outColor(ii, :) = colorValuesUint(idx, :);
            case 'int16'
                colorValuesInt16 = im2int16(colorValuesFloat);
                outColor(ii, :) = colorValuesInt16(idx, :);
        end
    end
end
%==========================================================================
function label = manageMissingLabels(label)
% Manage what to insert for missing strings or categorical labels. Follow
% the behavior of disp and display "<missing>" for strings and
% <"undefined"> for categoricals.
if ~isempty(label)
    if isstring(label)
        missingString = "<missing>";
    else
        missingString = "<undefined>";
    end
    label = fillmissing(string(label),'constant',missingString);
end
%==========================================================================
function numShapes = numberOfShapes(position)

if ndims(position) == 3
    % Compute the number of cuboids.
    % Vertices representation : 8-by-2-M.
    numShapes = size(position,3);
elseif all(size(position, [1,2]) == [8 2])
    % Compute the number of cuboids.
    % Vertices representation : 8-by-2-by-1
    numShapes  = 1; 
else
    % Compute the number of circles, rectangles and projected cuboids.
    % rectangles representation : M-by-8
    numShapes = size(position, 1);
end 
