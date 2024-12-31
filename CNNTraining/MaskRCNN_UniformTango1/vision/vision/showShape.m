function showShape(shape,position,params)
%

% Copyright 2020-2024 The MathWorks, Inc.

arguments
    shape (:,:)
    position (:,:,:)
    
    params.Color = lines(1)
    
    params.LineColor  = 'auto'
    
    params.LineWidth = 'auto'
    
    params.LineOpacity {...
        mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative, ...
        mustBeNonsparse, mustBeLessThanOrEqual(params.LineOpacity,1)} = 1
    
    params.Label = []
    
    params.LabelOpacity {...
        mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative, ...
        mustBeNonsparse, mustBeLessThanOrEqual(params.LabelOpacity,1)} = 1
    
    params.LabelTextColor = 'black'
    
    params.LabelFont = 'Helvetica';
    
    params.LabelFontSize {...
        mustBeNumeric, mustBeNonempty, mustBeReal, ...
        mustBeFinite, mustBeNonnegative, mustBeNonsparse} = 12
    
    params.Opacity {...
        mustBeNumeric, mustBeReal, mustBeFinite, mustBeNonnegative, ...
        mustBeNonsparse, mustBeLessThanOrEqual(params.Opacity,1)} = 0
    
    params.Parent = [];

    params.ShowOrientation
end

shape = iValidateShape(shape);
position = iValidateAndManagePosition(position, shape);

% Extract the number of shapes from the input position.
numShapes = computeNumShapes(position, shape);



params.Color           = iValidateAndManageColor(params.Color,numShapes,'Color');
params.Label           = iValidateAndManageLabels(params.Label,numShapes);
params.Opacity         = iValidateAndManageOpacity(params.Opacity,numShapes,'Opacity');
params.LineWidth       = iValidateAndManageLineWidth(params.LineWidth,numShapes);
params.LineColor       = iValidateAndManageLineColor(params.LineColor,numShapes,params.Color);
params.LineOpacity     = iValidateAndManageOpacity(params.LineOpacity,numShapes,'LineOpacity');
params.LabelOpacity    = iValidateAndManageOpacity(params.LabelOpacity,numShapes,'LabelOpacity');
params.LabelTextColor  = iValidateAndManageColor(params.LabelTextColor,numShapes,'LabelTextColor');
params.Font            = iValidateAndManageFontInfo(params.LabelFont,params.LabelFontSize);
params.Parent          = iValidateAndManageParent(params.Parent);
if isfield(params,'ShowOrientation')
    params.ShowOrientation = iValidateAndManageShowOrientation(params.ShowOrientation,shape,position);
else
    % By default, this is false. 
    params.ShowOrientation = false;
end

% Return an AxesShapeManager to use for shape management.
amngr = iMultiAxesManager(params.Parent);

% Support hold on/off. Replace ROIs when hold is off. Otherwise, existing
% ROIs remain.
if ~ishold(params.Parent)
    clearShapes(amngr);
end

% Only show shapes if there is something to display.
if ~isempty(position)
% Get shapes to show.
shapeType = iShapeType(shape,position);
shapes    = getShapes(amngr,shapeType,numShapes);

% Show and render the shapes.
show(shapes,position,params);
end

drawnow limitrate
end

%--------------------------------------------------------------------------
function colors = iValidateAndManageColor(colors,numShapes,paramName)
% Validate and manage colors. Return colors as floating point RGB triplets.
validateattributes(colors,{'numeric','string','cell','char'},{},paramName,mfilename);

if isnumeric(colors)
    
    iValidateNumericColor(colors);
    
    % Numeric color input can be double, single, uint8, uint16, or
    % int16. Convert to values in range [0 1].
    colors = im2double(colors);
    if isrow(colors)
        colors = repelem(colors,numShapes,1);
    end
    
elseif isstring(colors) || ischar(colors) || iscellstr(colors)
    
    colors = string(colors);
    if isscalar(colors)
        colors = repelem(colors,numShapes,1);
    end
    colors = iConvertToNumericColor(colors);
else
    error(message('vision:showShape:invalidColorFormat',paramName));
end

iValidateNumValuesEqualsNumShapes(size(colors,1),numShapes,paramName);

end

%--------------------------------------------------------------------------
function color = iConvertToNumericColor(colorString)
persistent colorSpecConverter
if isempty(colorSpecConverter)
    colorSpecConverter = images.internal.ColorSpecToRGBConverter;
end
m = numel(colorString);
color = zeros(m,3);
for ii = 1:m
    color(ii,:) = colorSpecConverter.convertColorSpec(colorString(ii));
end
end

%--------------------------------------------------------------------------
function lineColor = iValidateAndManageLineColor(lineColor,numShapes,color)

if ischar(lineColor) || (isstring(lineColor) && isscalar(lineColor))
    
    if strncmpi('auto',lineColor,strlength(lineColor))
        % LineColor is same as Color.
        lineColor = color;
        return;
    end
    
end

if isempty(lineColor) && numShapes == 0
    % LineColor is same as Color.
    lineColor = color;
else
    lineColor = iValidateAndManageColor(lineColor,numShapes,'LineColor');
end
end

%--------------------------------------------------------------------------
function iValidateNumValuesEqualsNumShapes(numValues,numShapes,paramName)
if numValues ~= numShapes
    error(message('vision:showShape:numValuesMustMatchNumShapes',paramName));
end
end

%--------------------------------------------------------------------------
function labels = iValidateAndManageLabels(labels,numShapes)

if isempty(labels)
    labels = "";
else
    validateattributes(labels,{'numeric','string','cell','categorical','char'},...
        {'nonsparse','vector'},mfilename,'Label');
    try
        % Convert label input into strings. Categorical values that are
        % undefined are converted to "<undefined>".
        labels = string(labels);
        labels(ismissing(labels)) = "<undefined>";
    catch
        error(message('vision:showShape:invalidLabel'));
    end
end
if isscalar(labels)
    labels = repelem(labels,numShapes,1);
end

iValidateNumValuesEqualsNumShapes(numel(labels),numShapes,'Label');

end
%--------------------------------------------------------------------------

function font = iValidateAndManageFontInfo(name,sz)
font = matlab.graphics.general.Font('Name',name);
if ~isempty(sz)
    % Set size, otherwise use default from HG.
    font.Size = sz;
end
end

%--------------------------------------------------------------------------
function pools = iAllocateShapePools()
sets = iMakeSupportedShapeSets();
pools = vision.internal.shape.ShapePool.createShapePools(sets);
end

%--------------------------------------------------------------------------
function [mngr] = iMultiAxesManager(ax)

% Setup persistent variables to hold object pools.
persistent AxesManager AxesList Pool
if isempty(AxesManager)
    % Allocate shape pools for all shapes supported by showShape. Each
    % shape pools is shared by multiple axes.
    Pool = iAllocateShapePools(); 
    
    % AxesList is an array of axes that shapes from the pool are currently
    % being displayed. 
    AxesList = gobjects(0);
    
    % AxesManager is a cell array of AxesShapeManager objects that manage
    % the shapes being shown in a specific axes. The axes manager handles
    % returning shape objects to the pool when axes are deleted.
    AxesManager = cell(1,0);
end

% Remove deleted axes from AxesList.
AxesList = iPruneDeletedAxes(AxesList);

% Find the axes into which showShape will display the shapes.
idx = find(ax == AxesList,1,'first');
if isempty(idx)
    % Set up an AxesManager for a new axes. Find a free slot in the
    % AxesList and reuse the AxesManager.
    availableSlot = arrayfun(@(x)isa(x,'matlab.graphics.GraphicsPlaceholder'),AxesList);
    availableSlot = find(availableSlot,1,'first');
    if isempty(availableSlot)
        % Unable to find an empty slot. Create a new AxesShapeManager.
        AxesList(end+1) = ax;
        AxesManager{end+1} = iMakeAxesManager(ax,Pool);
        idx = numel(AxesManager);
    else
        % Reuse mngr from a deleted axes and connect input ax to the
        % AxesShapeManager.
        AxesList(availableSlot) = ax;
        idx = availableSlot;
        connectToAxes(AxesManager{idx},ax);
    end
end
mngr = AxesManager{idx};
end

%--------------------------------------------------------------------------
function [axesList] = iPruneDeletedAxes(axesList)
deleted = ~ishandle(axesList);
axesList(deleted) = gobjects(1);
end

%--------------------------------------------------------------------------
function amngr = iMakeAxesManager(a,pool)
sets = iMakeSupportedShapeSets();
amngr = vision.internal.shape.AxesShapeManager.axesShapeManagerWithGlobalPool(a,sets,pool);
end

%--------------------------------------------------------------------------
function sets = iMakeSupportedShapeSets()
% Return the type of ShapeSets supported by this function.
import vision.internal.shape.*
sets = {
    RectangleSet
    RotatedRectangleSet
    CuboidSet
    PolygonSet
    LineSet
    CircleSet
    ProjectedCubeSet
    EllipseSet
    };
end

%--------------------------------------------------------------------------
function type = iShapeType(shape,position)
if shape == "rectangle"
    if size(position,2) == 5
        type = vision.internal.shape.ShapeType.RotatedRectangle;
    else
        type = vision.internal.shape.ShapeType.Rectangle;
    end
else
    type = vision.internal.shape.ShapeType.fromName(shape);
end
end

%--------------------------------------------------------------------------
function opacity = iValidateAndManageOpacity(opacity,numShapes,paramName)
opacity = iExpandScalarToVector(opacity,numShapes);
iValidateNumValuesEqualsNumShapes(numel(opacity),numShapes,paramName);
end

%--------------------------------------------------------------------------
function lineWidth = iValidateAndManageLineWidth(lineWidth,numShapes)

if ischar(lineWidth) || (isstring(lineWidth) && isscalar(lineWidth))
    validatestring(lineWidth, {'auto'}, mfilename, 'LineWidth');
    % Use default value for LineWidth
    lineWidth = images.roi.internal.getLineSize();
    
    lineWidth = iExpandScalarToVector(lineWidth,numShapes);
    iValidateNumValuesEqualsNumShapes(numel(lineWidth),numShapes,'LineWidth');
end

if isempty(lineWidth) && numShapes == 0
    % Fill with the default line size of 3 times number of points per
    % screen pixel.
    lineWidth = images.roi.internal.getLineSize();
    
    lineWidth = iExpandScalarToVector(lineWidth,numShapes);
else
    validateattributes(lineWidth,{'numeric'}, {'real','finite','nonnegative','nonsparse',},...
        mfilename, 'LineWidth');
    
    lineWidth = iExpandScalarToVector(lineWidth,numShapes);
    iValidateNumValuesEqualsNumShapes(numel(lineWidth),numShapes,'LineWidth');
end

end
%--------------------------------------------------------------------------
function num = computeNumShapes(pos, shape)
if strcmp(shape,'projected-cuboid') && all(size(pos, [1,2]) == [8,2])
    % Vertices representation of projected cuboid : 8-by-2-sz
    num = size(pos, 3);
else
    num = size(pos, 1);
end 
end
%--------------------------------------------------------------------------
function x = iExpandScalarToVector(x,n)
if isscalar(x)
    x = repelem(x,n,1);
end
end

%--------------------------------------------------------------------------
function iValidateNumericColor(color)
attrb = {'nonnegative','finite','size',[NaN 3],'real','nonsparse'};
if isfloat(color)
    % Floating point values must be within [0 1].
    attrb = [attrb '<=', 1];
end
validateattributes(color,{'double','single','uint8','uint16','int16'},...
    attrb,'Color',mfilename);
end

%--------------------------------------------------------------------------
function p = iValidateAndManageParent(p)
if isempty(p)
    p = gca;
else
   vision.internal.inputValidation.validateAxesHandle(p);
end
end

%--------------------------------------------------------------------------
function showOrientation = iValidateAndManageShowOrientation(showOrientation, shape, position)
% Validate 'ShowOrientation'
vision.internal.inputValidation.validateLogical(showOrientation,"ShowOrientation")

% Error if the shape is not 'rectangle' and 'position' is not M-by-5
if ~strcmp(shape,'rectangle') || size(position,2) ~= 5
    error(message('vision:showShape:invalidShowOrientationUse'));
end

end

%--------------------------------------------------------------------------
function s = iValidateShape(s)
s = validatestring(s,{'rectangle','polygon','line','circle','cuboid','projected-cuboid','ellipse'},...
    mfilename,'shape');
s = string(s);
end

%--------------------------------------------------------------------------
function pos = iValidateAndManagePosition(pos,shape)
if ~isempty(pos)
    switch shape
        case "rectangle"
            iValidatePositionRectangle(pos);
        case {"polygon","line"}
            pos = iValidateAndManagePositionPolygonOrLine(pos);
        case "circle"
            iValidatePositionCircle(pos);
        case "cuboid"
            iValidatePositionCuboid(pos);
        case "projected-cuboid"
            iValidatePositionProjectedCuboid(pos);
        case "ellipse"
            iValidatePositionEllipse(pos);
    end
end
end

%--------------------------------------------------------------------------
function iValidatePositionRectangle(pos)
vision.internal.inputValidation.checkBBox(pos,mfilename,'position',2);
w = size(pos,2);
if ~(w == 4 || w == 5)
    error(message('vision:showShape:invalidRectangleFormat'));
end
end

%--------------------------------------------------------------------------
function iValidatePositionCuboid(pos)
vision.internal.inputValidation.checkBBox(pos,mfilename,'position',2);
if size(pos,2) ~= 9
    error(message('vision:showShape:invalidCuboidFormat'));
end
end

%--------------------------------------------------------------------------
function iValidatePositionProjectedCuboid(pos)
validVertex = all(size(pos, [1,2]) == [8, 2]);
validRectangular = (size(pos, 2) == 8);
anyValid = validVertex || validRectangular;
if ~anyValid  
    error(message('vision:showShape:invalidProjectedCuboidFormat'));
end
end

%--------------------------------------------------------------------------
function iValidatePositionCircle(pos)
validateattributes(pos,{'numeric'},{'real','nonsparse','finite','size',[NaN 3]},...
    mfilename,'position');
if any(pos(:,3) <= 0)
    error(message('vision:showShape:invalidCircleFormat'));
end
end

%--------------------------------------------------------------------------
function iValidatePositionEllipse(pos)
validateattributes(pos,{'numeric'},{'real','nonsparse','finite','size',[NaN 5]},...
    mfilename,'position');
if (any(pos(:,3) <= 0) || any(pos(:,4) <= 0)) || any(pos(:,3) < pos(:,4))
    error(message('vision:showShape:invalidEllipseFormat'));
end
end

%--------------------------------------------------------------------------
function pos = iValidateAndManagePositionPolygonOrLine(pos)
% Validate and manage the various input formats of the polygon/line
% position. Return position values as a cell array of M-by-2 matrices.
validateattributes(pos,{'cell','numeric'},{},mfilename,'position');
if iscell(pos)
    for ii = 1:numel(pos)
        pos{ii} = iValidateAndManagePositionOfEachPolygonOrLine(pos{ii});
    end
else
    pos = {iValidateAndManagePositionOfEachPolygonOrLine(pos)};
end
end

%--------------------------------------------------------------------------
function pos = iValidateAndManagePositionOfEachPolygonOrLine(pos)
% Validate polygon and line position values are return as an M-by-2 matrix
% of [x y] vertices.
%
% The input position can be M-by-2 or 1-by-2L. 

validateattributes(pos,{'numeric'},...
    {'real','finite','nonsparse'},mfilename,'position');

isValidRowFormat = isrow(pos) && mod(size(pos,2),2)==0;
isValidMatFormat = ismatrix(pos) && size(pos,2) == 2;

if ~(isValidRowFormat || isValidMatFormat)
    error(message('vision:showShape:invalidPolygonAndLineFormat'))
end

if isValidRowFormat
    % Convert from 1-by-2L ([x1 y1 x2 y2 ...]) to M-by-2.
    pos = reshape(pos,2,[])';
end
end
%==========================================================================
