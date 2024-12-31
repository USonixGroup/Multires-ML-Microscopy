function RGB = insertShape(I, shape, position, varargin)
%

% Copyright 2019-2024 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

if isSimMode()
    if nargin > 3
        [varargin{:}] = convertStringsToChars(varargin{:});
    end
end

%% == Parse inputs and validate ==
[tmpRGB, shapeOut, fillShape, positionOut, ...
    lineWidth, color, opacity, smoothEdges, isEmpty]= ...
    validateAndParseInputs(I, shape, position, varargin{:});

positionOut = removeAdjacentSamePts(positionOut, shape);
fillShape = coder.internal.const(fillShape);


% handle empty I or empty position
if isEmpty
    RGB = tmpRGB;
    return;
end

%% == Call Built-in Function ==
dtClass = coder.internal.const(class(I));
if isSimMode()
    RGB = shapeInserter(shape, shapeOut, fillShape, lineWidth, opacity,...
        smoothEdges, dtClass, tmpRGB, positionOut, color);
else
    RGB = shapeInserterCG(shape, shapeOut, fillShape, lineWidth, opacity,...
        smoothEdges, dtClass, tmpRGB, positionOut, color);
end


function positionOut = removeAdjacentSamePtsInVector(position, isPolygon)

N = length(position)/2;
position = reshape(position, [2 N]);
coder.varsize('pos');
pos = position;
if isPolygon
    minValidN = 3; % 6 flattened columns (3 reshaped column)
else % line
    minValidN = 2; % 4 flattened columns (2 reshaped column)
end
if (N > minValidN) % no need to eliminate any point from minimum set
    px = pos(1,:);
    py = pos(2,:);
    
    diff_px = diff(px);
    diff_py = diff(py);
    
    logicalIdx_RepeatedXY = [(diff_px == 0) & (diff_py == 0) false];
    
    if isPolygon % remove last point if it is same as the first point
        logicalIdx_RepeatedXY(end) = logicalIdx_RepeatedXY(end) || ...
            ((px(1) == px(end))  && ...
            (py(1) == py(end)));
    end
    % numXsToRemove is same as numYsToRemove
    numXsToRemove = length(logicalIdx_RepeatedXY(logicalIdx_RepeatedXY == 1));
    if  (N-numXsToRemove) >= minValidN
        pos(:, logicalIdx_RepeatedXY) = [];
    end
end
positionOut = pos(:);

%==========================================================================
function positionOut = removeAdjacentSamePts(position, shape)

isPolygon = strcmp(shape, 'Polygon');
if (strcmp(shape, 'Line') || isPolygon)
    if isvector(position)
        % no need to convert to cell array
        positionOut = removeAdjacentSamePtsInVector(position, isPolygon);
    elseif ismatrix(position)
        numShapes = size(position, 1);
        coder.varsize('pos');
        coder.varsize('positionOut');
        
        posTmp = cell(numShapes, 1);
        pos = coder.nullcopy(posTmp);
        for i= 1: numShapes
            pos{i} = removeAdjacentSamePtsInVector(position(i,:), isPolygon);
        end
        positionOut = zeros(size(position), 'int32');
        
        maxCellLength = size(position, 2);
        
        for ii = 1:numShapes
            positionOut(ii,:) = copyCellAndRepeatLastPoint(...
                int32(pos{ii}), maxCellLength);
        end
    elseif iscell(position)
        numShapes = length(position);
        coder.varsize('pos');
        coder.varsize('positionOut');
        
        posTmp = cell(numShapes, 1);
        pos = coder.nullcopy(posTmp);
        for i = 1: numShapes
            pos{i} = removeAdjacentSamePtsInVector(position(i,:), isPolygon);
        end
        
        maxCellLength = getMaxCellLength(position);
        positionOut = zeros([numShapes maxCellLength],'int32');
        
        for ii = 1: numShapes
            positionOut(ii,:) = copyCellAndRepeatLastPoint(...
                int32(pos{ii}), maxCellLength);
        end
    end
else
    positionOut = position;
end


%==========================================================================
function [RGB, shapeOut, fillShape, positionOut, lineWidth, colorOut, opacity, ...
    smoothEdges, isEmpty] = validateAndParseInputs(I, shape, position, varargin)

%--input image--
checkImage(I);
RGB = convert2RGB(I);
inpClass = coder.internal.const(class(I));

%--shape--
if ~isSimMode()
    eml_invariant(eml_is_const(shape),...
        eml_message('vision:insertShape:shapeNonConst'));
end
shape1 = validatestring(shape,{'rectangle', 'filled-rectangle', ...
    'line', 'polygon', 'filled-polygon', 'circle', 'filled-circle', ...
    'projected-cuboid','ellipse', 'filled-ellipse', ...
    'FilledRectangle','FilledPolygon','FilledCircle'},...
    'insertShape', 'SHAPE', 2);
shape2 = coder.internal.const(shape1);

%--position--
checkPosition(position, shape1);

%--isEmpty--
isEmpty = anyEmpty(I, position);

%--other optional parameters--
if isEmpty
    shapeOut    = shape2;
    fillShape   = coder.internal.const(true); % not used
    positionOut = int32(position);
    lineWidth   = 1;
    colorOut    = ones(1, 3, inpClass); % not used
    opacity     = 0.6;% not used
    smoothEdges = true; % not used
else
    % lineWidth ignored for ~fillShape
    [lineWidth, color, opacity, smoothEdges] = ...
        validateAndParseOptInputs(inpClass, varargin{:});
   
    % Convert ProjectedCuboid to Line object
    %
    % projected cuboids are not treated as separate objects, rather as a
    % set of 6 line objects.

    if strcmp(shape1, 'projected-cuboid')
        numProjCuboid = numberOfProjectedCuboids(position);
        shape2 = coder.internal.const('line');
        positionNew = validateAndConvertProjectedCuboid(position, numProjCuboid);
        color2 = colorForProjectedCuboid(color);
    else
        positionNew = position;
        color2 = color;
    end

    crossCheckInputs(shape2, positionNew, color2);
    
    [shapeOut, fillShape] = remapShape(shape1);
    
    positionOut = convertPositionToMatrix(positionNew, shapeOut); 
    numPosition = size(positionOut, 1);
    colorOut = getColorMatrix(inpClass, numPosition, color2);
end

%==========================================================================
function isAnyEmpty = anyEmpty(I, position)
% for fixS, check max size
% for varS, check run-time size

if isSimMode()
    isAnyEmpty = isempty(I) || isempty(position);
else
    isAnyEmpty = false;% no suitable function
end

%==========================================================================
function checkPosition(position, shape)
% Validate label

if isnumeric(position)
    % numeric objects not supported. 
    vision.internal.inputValidation.validateNotObject(position,'vision','position');
    if strcmp(shape,'projected-cuboid')
        % Projected cuboids positions can have 3D shape.
        validateattributes(position, {'numeric'}, ...
            {'real', 'nonsparse','finite'}, ...
            'insertShape', 'POSITION');
    else
        validateattributes(position, {'numeric'}, ...
            {'real', 'nonsparse','2d','finite'}, ...
            'insertShape', 'POSITION');
    end
else
    if ~isSimMode()
        % codegen does not support cell array
        errIf0(~isnumeric(position), 'vision:insertShape:posNotNumeric');
    else
        validateattributes(position,{'cell'}, {'nonempty', 'vector'}, ...
            'insertShape', 'POSITION');
        numCell = length(position);
        for ii = 1:numCell
            % numeric objects not supported.  
            vision.internal.inputValidation.validateNotObject(position{ii},'vision','position');
            
            % each cell must be numeric and matrix
            errCond = ~isnumeric(position{ii}) || ~ismatrix(position{ii});
            errIf0(errCond, 'vision:insertShape:posCellNonNumMtx');
            
            % each cell (a vector) must be non-empty
            errCond = isempty(position{ii});
            errIf0(errCond, 'vision:insertShape:positionCellEmpty');
        end
    end
end

%==========================================================================
function [lineWidth, color, opacity, smoothEdges] = ...
    validateAndParseOptInputs(inpClass, varargin)

% Validate and parse optional inputs
defaults = getDefaultParameters(inpClass);
if nargin>1 % varargin{:} is non-empty
    if (isSimMode())
        % Setup input parser
        parser = inputParser;
        parser.CaseSensitive = false;
        parser.FunctionName  = 'insertShape';
        
        parser.addParameter('LineWidth', defaults.LineWidth);
        parser.addParameter('Color', defaults.Color); % old parameter kept for backward compatibility
        parser.addParameter('ShapeColor', defaults.Color);
        parser.addParameter('Opacity', defaults.Opacity);
        parser.addParameter('SmoothEdges', defaults.SmoothEdges);
        
        %Parse input
        parser.parse(varargin{:});
        
        lineWidth   = parser.Results.LineWidth;
        color       = vision.internal.parseInserterColor("Color", "ShapeColor", parser, mfilename, inpClass);
        opacity     = parser.Results.Opacity;
        smoothEdges = parser.Results.SmoothEdges;
        
    else
        
        % Define parser mapping struct
        pvPairs = struct(...
            'LineWidth',   uint32(0), ...
            'Color',       uint32(0), ... % old parameter kept for backward compatibility
            'ShapeColor',  uint32(0), ...
            'Opacity',     uint32(0),...
            'SmoothEdges', uint32(0));
        
        % Specify parser options
        poptions = struct( ...
            'CaseSensitivity', false, ...
            'StructExpand',    true, ...
            'PartialMatching', true);
        
        % Parse PV pairs
        pstruct = coder.internal.parseParameterInputs(pvPairs, ...
            poptions, varargin{:});
        % Extract inputs        
        lineWidth  = coder.internal.getParameterValue(pstruct.LineWidth, defaults.LineWidth, varargin{:});        
        color = vision.internal.codegen.parseInserterColor('Color', 'ShapeColor', ...
            pstruct, coder.const(mfilename), inpClass, defaults, varargin{:});

        % opacity and smoothEdges (i.e., useAntiAliasing) are input mask params
        % (not from port); that's why convert these to eml_const at parsing stage
        
        opacity    = coder.internal.getParameterValue(pstruct.Opacity, defaults.Opacity, varargin{:});
        smoothEdges = coder.internal.const(tolower(coder.internal.getParameterValue( ...
            pstruct.SmoothEdges, defaults.SmoothEdges, varargin{:})));
        
    end
    
    checkLineWidth(lineWidth);
    checkOpacity(opacity);
    checkSmoothEdges(smoothEdges);
    
    lineWidth   = double(lineWidth);
    opacity     = double(opacity);
    smoothEdges = logical(smoothEdges);
    
else % varargin{:} is empty (no name-value pair)
    lineWidth   = defaults.LineWidth;
    color       = defaults.Color;
    opacity     = defaults.Opacity;
    smoothEdges = defaults.SmoothEdges;
end

%==========================================================================
function flag = isSimMode()

flag = isempty(coder.target);

%==========================================================================
function checkImage(I)
% Validate input image

% No objects allowed.
vision.internal.inputValidation.validateNotObject(I, 'vision', 'I');

validateattributes(I, {'uint8', 'uint16', 'int16', 'double', 'single'}, ...
    {'real','nonsparse'}, 'insertShape', 'I', 1)

% input image must be 2d or 3d (with 3 planes)
errIf0((ndims(I) > 3) || ((size(I,3) ~= 1) && (size(I,3) ~= 3)), 'vision:dims:imageNot2DorRGB');

%==========================================================================
function crossCheckInputs(shape, position, color)
% Cross validate inputs

crossCheckShapePosition(shape, position);
crossCheckPositionColor(position, color);

%==========================================================================
function crossCheckPositionColor(position, color)
% Cross validate inputs
% here position must be non-empty

if isnumeric(position)
    numPositions = size(position, 1);
    msgID = 'vision:insertShape:invalidNumPosMatrixNumColor';
else
    numPositions = length(position);
    msgID = 'vision:insertShape:invalidNumPosCellNumColor';
end
numColors = getNumColors(color);
errCond = (numColors ~= 1) && (numPositions ~= numColors);
errIf0(errCond, msgID);

%==========================================================================
function crossCheckShapePosition(shape, position)
% Cross validate inputs
% here position must be non-empty

switch shape
    case {'rectangle' ,'filled-rectangle', 'Rectangle', 'FilledRectangle'}
        errCond = iscell(position);
        errIf1(errCond, 'vision:insertShape:positionCell', shape);

        % rectangle may be either axis-aligned [x y width height] or a
        % rotated rectangle [xctr yctr width height yaw]
        posSize = size(position, 2);
        errCond = posSize ~= 4 && posSize ~= 5;
        errIf1(errCond, 'vision:insertShape:posColsNot4ForRect', shape);
        
    case {'circle','filled-circle', 'Circle', 'FilledCircle'}
        errCond = iscell(position);
        errIf1(errCond, 'vision:insertShape:positionCell', shape);
        
        errCond = size(position, 2) ~= 3; % circle: [x y radius]
        errIf1(errCond, 'vision:insertShape:posColsNot4ForCir', shape);
    
    case {'ellipse','filled-ellipse'}      
        
        if iscell(position)
            errIf1(true, 'vision:insertShape:positionCell', shape);
        else
            errCond = size(position, 2) ~= 5; % ellipse: [x y major minor yaw]
            errIf1(errCond, 'vision:insertShape:posColsNot4ForEllipse', shape);

            % Major axis must be >= minor axis.
            errIf0(any(position(:,3) < position(:,4)),...
                'vision:insertShape:invalidEllipseMajorMinorAxisLength');
        end
        
    otherwise % Line, Polygon, FilledPolygon
        if isnumeric(position)
            % Validate that number of columns is even.
            numCols = size(position,2);
            errCond = mod(numCols, 2) ~= 0;
            errIf1(errCond, 'vision:insertShape:posPolyNumPtsNotEven', shape);
            
            % Valid polygon or line position is M-by-2L or L-by-2.
            numMinPts = getMinNumPoints(shape);
            minL = numMinPts/2;
            
            if numCols == 2 
                % L-by-2 format.
                %   For line, minimum L must be 2.
                %   For polygon, minimum L must be 3.
                
                % Use coder.internal.error to allow variable size position
                % inputs
                if size(position, 1) < minL
                    coder.internal.error(...
                        'vision:insertShape:posPolyNumPtsLT2', ...
                        minL, shape);
                end
            else
                % 1-by-2L format.
                % For line: minimum 2 points (4 values: x1 y1 x2 y2)
                % For polygon: minimum 3 points (6 values: x1 y1 x2 y2 x3 y3)
                errCond = size(position, 2) < numMinPts;
                errIf2(errCond, 'vision:insertShape:posPolyNumPtsLT2', ...
                    minL, shape);
            end
            
            
        else % must be cell of numeric vectors
            errCond = ~isvector(position);
            errIf0(errCond, 'vision:insertShape:posPolyCellNumPtsOdd');
            
            numCell = length(position);
            for ii = 1:numCell
                cell_ii = position{ii};
                
                if isvector(cell_ii)
                    len = length(cell_ii);
                else
                    len = size(cell_ii,2);
                end
                
                % each cell must have even numbered elements
                errCond = mod(len, 2) ~= 0;
                errIf0(errCond, 'vision:insertShape:posPolyCellNumPtsOdd');
                
                numMinPts = getMinNumPoints(shape);
                if isrow(cell_ii)
                    % 1-by-2L format.
                    errCond = length(cell_ii) < numMinPts;
                    errIf2(errCond, 'vision:insertShape:posPolyCellNumPtsLT2', ...
                        numMinPts, shape);
                else
                    % L-by-2 format.
                    minL = numMinPts/2;
                    errCond = size(cell_ii, 1) < minL || size(cell_ii, 2) ~= 2;
                    errIf2(errCond, 'vision:insertShape:posPolyCellNumPtsLT2', ...
                        minL, shape);
                end
                
            end
        end
end

%==========================================================================
function numMinPts = getMinNumPoints(shape)

if strcmpi(shape, 'Line')
    numMinPts = 4;
else
    numMinPts = 6;
end
%==========================================================================
function colorOut = getColorMatrix(inpClass, numShapes, color)

colorRGB = vision.internal.convertColorSpecToRGB(color, inpClass);
if (size(colorRGB, 1) == 1)
    colorOut = repmat(colorRGB, [numShapes 1]);
else
    colorOut = colorRGB;
end

%==========================================================================
function numColors = getNumColors(color)

% Get number of colors
numColors = 1;
if isnumeric(color)
    numColors = size(color, 1);
elseif iscell(color) % if color='red', it is converted to cell earlier
    numColors = length(color);
end

%==========================================================================
function defaults = getDefaultParameters(inpClass)

% Get default values for optional parameters

% default color 'yellow'
switch inpClass
    case {'double', 'single'}
        yellow = [1 1 0];
    case 'uint8'
        yellow = [255 255 0];
    case 'uint16'
        yellow = [65535  65535  0];
    case 'int16'
        yellow = [32767  32767 -32768];
end

defaults = struct(...
    'LineWidth',   1, ...
    'Color',       yellow, ...
    'ShapeColor',  yellow, ...
    'Opacity',     0.6,...
    'SmoothEdges', true);

%==========================================================================
function str = tolower(str)
% convert a string to lower case

if isSimMode()
    str = lower(str);
else
    str = eml_tolower(str);
end

%==========================================================================
function tf = checkLineWidth(lineWidth)
% Validate 'LineWidth'
vision.internal.inputValidation.validateNotObject(lineWidth,'vision','LineWidth');
validateattributes(lineWidth, {'numeric'}, ...
    {'nonsparse', 'integer', 'scalar', 'real', 'positive'}, ...
    'insertShape', 'LineWidth');

tf = true;

%==========================================================================
function tf = checkOpacity(opacity)
% Validate 'TextBoxOpacity'
vision.internal.inputValidation.validateNotObject(opacity,'vision','Opacity');
validateattributes(opacity, {'numeric'}, {'nonempty', 'nonnan', ...
    'finite', 'nonsparse', 'real', 'scalar', '>=', 0, '<=', 1}, ...
    'insertShape', 'TextBoxOpacity');
tf = true;

%==========================================================================
function tf = checkSmoothEdges(smoothEdges)
% Validate 'SmoothEdges'
vision.internal.inputValidation.validateNotObject(smoothEdges,'vision','SmoothEdges');
validateattributes(smoothEdges, {'logical'}, {'scalar'}, 'insertShape', ...
    'SmoothEdges');
tf = true;


%==========================================================================
function rgb = convert2RGB(I)

if ismatrix(I)
    rgb = cat(3, I, I, I);
else
    rgb = I;
end

%==========================================================================
function maxCellLength = getMaxCellLength(inp)

maxCellLength = 0;
for i = 1:length(inp)
    thisCellLen = prod(size(inp{i}), 'all');
    if maxCellLength < thisCellLen
        maxCellLength = thisCellLen;
    end
end

%==========================================================================
function OutPosition = convertPositionToMatrix(inPosition,shapeEnum)

if isnumeric(inPosition)
    if shapeEnum == 5    
        % convert ellipse major/minor axes lengths to semi-major axes
        % lengths required by the underlying drawing routines.
        if isfloat(inPosition)
            inPosition(:,3:4) = 0.5*inPosition(:,3:4);
        else
            inPosition(:,3:4) = cast(0.5*single(inPosition(:,3:4)),'like',inPosition);
        end
        OutPosition = convertToPositionVectorFormat(inPosition);
    else
        OutPosition = convertToPositionVectorFormat(int32(inPosition));
    end
else
    numCell       = length(inPosition);
    maxCellLength = getMaxCellLength(inPosition);
    OutPosition   = zeros(numCell, maxCellLength, 'int32');
    for ii = 1:numCell
        vecPos = convertToPositionVectorFormat(int32(inPosition{ii}));
        OutPosition(ii,:) = copyCellAndRepeatLastPoint(...
            vecPos, maxCellLength);
    end
end

%%=========================================================================
function outPosition = convertToPositionVectorFormat(inPosition)
% Convert L-by-2 position format into 1-by-2L position vector format if
% needed.
if size(inPosition,2) == 2
    % L-by-2 format
    outPosition = reshape(inPosition',1,[]);
else
    outPosition = inPosition;
end
%==========================================================================
function out = copyCellAndRepeatLastPoint(in, outLen)

inLen = length(in);
out   = zeros(1, outLen, class(in));
out(1:inLen) = in;
for ii = (inLen+1): 2 : (outLen-1)
    out(ii) = in(end-1);
    out(ii+1) = in(end);
end

%==========================================================================
function [shapeOut, fillShape] = remapShape(shapeIn)

switch shapeIn
    case {'rectangle','Rectangle'}
        shapeOut = int32(1); %'Rectangles'
        fillShape = false;
    case {'FilledRectangle','filled-rectangle'}
        shapeOut = int32(1); %'Rectangles'
        fillShape = true;
    case {'line','Line'}
        shapeOut = int32(2); %'Lines'
        fillShape = false;
    case {'polygon','Polygon'}
        shapeOut = int32(3); %'Polygons'
        fillShape = false;
    case {'FilledPolygon','filled-polygon'}
        shapeOut = int32(3); %'Polygons'
        fillShape = true;
    case {'circle','Circle'}
        shapeOut = int32(4); %'Circles'
        fillShape = false;
    case {'FilledCircle','filled-circle'}
        shapeOut = int32(4); %'Circles'
        fillShape = true;
    case 'projected-cuboid'
        shapeOut = int32(2); %'Lines'
        fillShape = false;
    case {'ellipse'}
        shapeOut = int32(5); %'Ellipses'
        fillShape = false;
    case {'filled-ellipse'}
        shapeOut = int32(5); %'Ellipses'
        fillShape = true;
    otherwise
        % code-flow should not reach here.
        assert(false,'Incorrect shape choice.')
     
end

%==========================================================================
function out = convertProjectedCuboidToLines(in, isVertices)
% Convert projected cuboid 8x2 or 1x8 representation to lines
% connecting vertices.

if isempty(coder.target) && isa(in, 'cell')
    in = in{:};
end

in = squeeze(in);

if ~isVertices
    frontFacebbox = in(1:4);
    backFacebbox = in(5:8);
    frontPoints = bbox2points(frontFacebbox);
    backPoints = bbox2points(backFacebbox);
else
    frontPoints = in(1:4,:);
    backPoints = in(5:8,:);
end

% Break the projected cuboid into a set of 12 lines.
% connections represent lines connecting front and back faces.
% frontConnections and backConnections are lines making up the faces. 

connections1 = [frontPoints(1,:) backPoints(1,:) ];
connections2 = [frontPoints(2,:) backPoints(2,:) ];
connections3 = [frontPoints(3,:) backPoints(3,:) ];
connections4 = [frontPoints(4,:) backPoints(4,:) ];
frontConnections1 = [frontPoints(1,:) frontPoints(2,:)];
frontConnections2 = [frontPoints(2,:) frontPoints(3,:)];
frontConnections3 = [frontPoints(3,:) frontPoints(4,:)];
frontConnections4 = [frontPoints(4,:) frontPoints(1,:)];

backConnections1 = [backPoints(1,:) backPoints(2,:)];
backConnections2 = [backPoints(2,:) backPoints(3,:)];
backConnections3 = [backPoints(3,:) backPoints(4,:)];
backConnections4 = [backPoints(4,:) backPoints(1,:)];

out = {connections1 connections2 connections3 ...
     connections4 frontConnections1 frontConnections2 frontConnections3 ...
     frontConnections4 backConnections1 backConnections2 backConnections3 ....
     backConnections4};

%==========================================================================
function colorOut = colorForProjectedCuboid(color)
% If shape is projected cuboids, it is broken down into 12 sets of
% lines. The color needs to be copied twelve times for each projected
% cuboid.

isSingeRGB = all(size(color) == [1,3]);
isSingleChar = numel(color) == 1;
if isSingeRGB || isSingleChar % Ensure its not default value
    colorOut = color;
else
    if iscell(color)
        % Cell array with size = 6*number of cuboids
        temp = repelem(color,12,1);
        colorOut = reshape(temp,[],1)';
    else
        temp = repmat(color,1,12);
        colorOut = reshape(temp',3,[])';
    end
end

%==========================================================================
function out = cuboidToLinesCG(position, sz)

% Convert projected cuboid to lines. Each projected cuboid is broken down
% into a set of 12 lines.
out = cell(1, sz*12);
if ndims(position) == 3 || all(size(position,[1,2]) == [8 2])
    % Vertices representation. 
    
    isVertices = 1;
    lines = cell(1, sz);
    for i=1:sz
        lines{i} = convertProjectedCuboidToLines(squeeze(position(:,:,i)), isVertices);
    end
    
else
     % Rectangle representation. 
     isVertices = 0;
     lines = cell(1,sz);
     for i = 1:sz
         lines{i}  = convertProjectedCuboidToLines(position(i,:), isVertices);
     end         
end
% Assign to output. This manual operation is done to prevent the 
% "Unable to Determine That Every Element of Cell Array Is
% Assigned" issue.
for i=1:sz
    temp = lines{i};
    for j = 0:11
        out{i*12-11+j} = temp{j+1};
    end
end

%==========================================================================
function out = cuboidToLines(position)
% Convert projected cuboid to set of twelve lines.

if ndims(position) == 3 || all(size(position) == [8 2])
    % Vertices representation. 
    isVertices = 1;
    position = num2cell(position,[1 2]);
    position = squeeze(position);
    position = arrayfun(@(x) convertProjectedCuboidToLines(x, isVertices), position, 'UniformOutput', false);
    out = [position{:}];
else
    % Rectangle representation. 
    isVertices = 0;
    position = num2cell(position,2)';
    position = arrayfun(@(x) convertProjectedCuboidToLines(x, isVertices), position, 'UniformOutput', false);
    out = [position{:}];
end
%==========================================================================
function sz = numberOfProjectedCuboids(position)
% Compute the number of cuboids.

if ndims(position) == 3
    % Vertices representation : 8-by-2-sz
    sz = size(position,3);
elseif all(size(position, [1,2]) == [8 2])
    % Vertices representation : 8-by-2
    sz  = 1; 
else
    % rectangles representation : sz-by-8
    sz = size(position, 1);
end 
%==========================================================================
function outImage = shapeInserter(shapeIn, shapeOut, fillShape, lineWidth, opacity,...
        smoothEdges, dtClass, tmpRGB, positionOut, color)
% Insert shape on image. The projected-cuboid case is handeled as a set of
% two separate insertions to avoid line overlap issues (g2758220).
if strcmp(shapeIn, 'projected-cuboid')
    % Populate list of lines to insert in the first and second insertions.
    sz = size(positionOut,1);
    
    % Each projected cuboid is cast into set of 12 lines. Lines 5-8 and
    % 9-12 correspond to two faces of the cuboids and lines 1-4 correspond
    % to the connections between the faces. positionOut contains all the
    % line connections. connectionLines contains indices of lines 1-4 for 
    % projected cuboid and faceLines contains indices of the remaining
    % lines.
    connectionsTemporary = zeros(4,(sz/12));
    facesTemporary = zeros(8,(sz/12));    
    for i=1:4
        connectionsTemporary(i,:) = i:12:sz;
    end
    connectionLines = reshape(connectionsTemporary',1,[]);
    for i=5:12
        facesTemporary(i-4,:) = i:12:sz;
    end
    faceLines = reshape(facesTemporary',1,[]);

    % Insert the connection lines.
    tmpImage = visionInsertShape(int32(2), fillShape, lineWidth, opacity,...
    smoothEdges, dtClass, tmpRGB, positionOut(connectionLines,1:4), color(connectionLines,:));
    % Insert the face lines.
    tmpImage2 = visionInsertShape(int32(2), fillShape, lineWidth, opacity,...
    smoothEdges, dtClass, tmpImage, positionOut(faceLines,1:4), color(faceLines,:));

    outImage = tmpImage2;

elseif shapeOut == int32(1) && size(positionOut, 2) == 5
    % Rotated rectangles are handled as a polygon with four vertices
    % Convert rotated rectangle to vertices
    positionOut = convertRotatedRectangleToPoly(positionOut);

    % Insert rotated rectangle as a polygon
    outImage = visionInsertShape(int32(3), fillShape, lineWidth, opacity,...
    smoothEdges, dtClass, tmpRGB, positionOut, color);

elseif shapeOut == int32(5)
    color = double(color); 
    angle = double(positionOut(:,5));
    [center, axesLength, numBitsToShift] = iReformatEllipseParameters(positionOut);

    outImage = ocvInsertEllipse(tmpRGB, center, axesLength, angle, fillShape, lineWidth, ...
        color, opacity, smoothEdges, numBitsToShift);
else
    % For all other shapes.
    outImage = visionInsertShape(shapeOut, fillShape, lineWidth, opacity,...
    smoothEdges, dtClass, tmpRGB, positionOut, color);
end
%==========================================================================
function outImage = shapeInserterCG(shapeIn, shapeOut, fillShape, lineWidth, opacity,...
        smoothEdges, dtClass, tmpRGB, positionOut, color)
if strcmp(shapeIn, 'projected-cuboid')
    % Generate the list of lines 
    
    sz = size(positionOut,1);
    connectionsTemporary = zeros(4,(sz/12));
    facesTemporary = zeros(8,(sz/12));    
    for i=1:4
        connectionsTemporary(i,:) = i:12:sz;
    end
    connectionLines = reshape(connectionsTemporary',1,[]);
    for i=5:12
        facesTemporary(i-4,:) = i:12:sz;
    end
    faceLines = reshape(facesTemporary',1,[]);


    tmpImage = vision.internal.buildable.insertShapeBuildable.insertShape(int32(2), fillShape, lineWidth, opacity,...
    smoothEdges, dtClass, tmpRGB, positionOut(connectionLines,1:4), color(connectionLines,:));

    tmpImage2 = vision.internal.buildable.insertShapeBuildable.insertShape(int32(2), fillShape, lineWidth, opacity,...
    smoothEdges, dtClass, tmpImage, positionOut(faceLines,1:4), color(faceLines,:));
    outImage = tmpImage2;

elseif shapeOut == int32(1) && size(positionOut, 2) == 5
    % Rotated rectangles are handled as a polygon with four vertices
    % Convert rotated rectangle to vertices
    positionOut = convertRotatedRectangleToPoly(positionOut);

    % Insert rotated rectangle as a polygon
    outImage = vision.internal.buildable.insertShapeBuildable.insertShape(int32(3), fillShape, lineWidth, opacity,...
    smoothEdges, dtClass, tmpRGB, positionOut, color);

elseif shapeOut == int32(5)
    color = double(color); 
    angle = double(positionOut(:,5));
    
    [center, axesLength, numBitsToShift] = iReformatEllipseParameters(positionOut);
    
    % Insert ellipse using OpenCV API
    outImage = vision.internal.buildable.insertShapeEllipseBuildable.insertEllipse(...
        tmpRGB, center, axesLength, angle,...
        fillShape, lineWidth, ...
        color, opacity, smoothEdges, numBitsToShift);
else
    outImage = vision.internal.buildable.insertShapeBuildable.insertShape(shapeOut, fillShape, lineWidth, opacity,...
    smoothEdges, dtClass, tmpRGB, positionOut, color);
end

%==========================================================================
function [center, axesLength, numBitsToShift] = iReformatEllipseParameters(positionOut)
% Encode floating point center and axes length values as fixed point
% integers. Use a bit shift of 10 bits for the encoding. A shift value
% of 10 results in a error of 1e-5, which is negligible for
% visualization purposes.
%
% In addition, offset pixel coordinates to match OpenCV's image origin
% which starts at (0,0) compared to (0.5,0.5) in MATLAB.
if isfloat(positionOut)
    numBitsToShift = int32(10);
    factor = cast(2^numBitsToShift,"like",positionOut);
    center = int32((positionOut(:,1:2)-0.5)*factor);
    axesLength = int32(positionOut(:,3:4)*factor);
else
    numBitsToShift = int32(0);
    center = int32(positionOut(:,1:2)-ones(1,1,"like",positionOut));
    axesLength = int32(positionOut(:,3:4));
end

%==========================================================================
function positionOut = validateAndConvertProjectedCuboid(position, sz)
% Validate projected cuboids.

if ndims(position) == 3 || all(size(position) == [8 2])
    % Vertices representation : 8-by-2-sz
    validateattributes(position, ...
        {'numeric'},...
        {'real', 'nonsparse', 'nonnan', 'finite', 'size', [8 2 NaN]}, ...
        'insertShape', 'SHAPE');
else
    % rectangles representation : sz-by-8
    validateattributes(position, ...
        {'numeric'},...
        {'real', 'nonsparse', 'nonnan', 'finite', 'size', [NaN 8]}, ...
        'insertShape', 'SHAPE');
end
% Convert projected cuboid to polylines lines.
if isempty(coder.target)        
    positionOut = cuboidToLines(position);
else   
    positionOut = cuboidToLinesCG(position, sz);
end
    
%==========================================================================
function positionOut = convertRotatedRectangleToPoly(position)
% Convert rotated bounding boxes to vertices
[X,Y] = vision.internal.bbox.bbox2poly(cast(position,'double'));

% Convert 4-by-M X and Y to M-by-2L position format.
positionOut = zeros([size(X,2),2*size(X,1)],'like',position);
positionOut(:,1:2:end) = cast(X','like',position);
positionOut(:,2:2:end) = cast(Y','like',position);

%==========================================================================
function errIf0(condition, msgID)

coder.internal.errorIf(condition, msgID);

%==========================================================================
function errIf1(condition, msgID, strArg)

coder.internal.errorIf(condition, msgID, strArg);

%==========================================================================
function errIf2(condition, msgID, strArg1, strArg2)

coder.internal.errorIf(condition, msgID, strArg1, strArg2);


% LocalWords:  grayscale Xs Ys nonsparse Mtx PV Dor Pts Vec techdoc colorspec truecolor
