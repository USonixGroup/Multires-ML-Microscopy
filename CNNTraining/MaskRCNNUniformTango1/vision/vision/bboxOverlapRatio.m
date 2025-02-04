function overlapRatio = bboxOverlapRatio(bboxA, bboxB, varargin)
%bboxOverlapRatio Compute bounding box overlap ratio.
%  overlapRatio = bboxOverlapRatio(bboxA, bboxB) returns the overlap ratio
%  between each pair of bounding boxes contained in bboxA and bboxB.
%  Bounding boxes in bboxA and bboxB can be axis-aligned rectangles or
%  rotated rectangles. bboxA is an M1-by-N matrix and bboxB is an M2-by-N
%  matrix. Each row in bboxA and bboxB defines one bounding box. By
%  default, the overlap ratio between two boxes A and B is defined as
%  area(A intersect B) / area(A union B). The range of overlapRatio is
%  between 0 and 1, where 1 implies a perfect overlap.
%
%  The format used to define a bounding box and the length of each row in
%  bboxA and bboxB depends on the type of bounding box. An axis-aligned
%  bounding box is defined as [xmin ymin width height] and a rotated
%  bounding box is defined as [xcenter ycenter width height yaw], where yaw
%  represents the angle in degrees.
%
%  overlapRatio = bboxOverlapRatio(bboxA, bboxB, ratioType) additionally
%  lets you specify the method to compute the ratio. ratioType can be
%  'Union', as described above, or 'Min'. When ratioType is 'Min', the
%  overlap ratio is defined as area(A intersect B) / min(area(A),area(B)).
%
%  Class Support
%  -------------
%  bboxA and bboxB are real, finite, and nonsparse. They can be uint8,
%  int8, uint16, int16, uint32, int32, single and double. The output
%  overlapRatio is double if bboxA or bboxB is double, otherwise it is
%  single.
%
%  Example 1: Overlap ratio between two bounding boxes.
%  ----------------------------------------------------
%  % Define two bounding boxes.
%  bboxA = [1, 1, 100, 200];
%  bboxB = bboxA + 20;
%
%  % Compute the overlap ratio between the two bounding boxes.
%  overlapRatio = bboxOverlapRatio(bboxA, bboxB)
%
%  Example 2: Pairwise overlap ratio between sets of bounding boxes.
%  -----------------------------------------------------------------
%  % Generate two sets of rotated bounding boxes.
%  bboxA = 10*rand(5, 5);
%  bboxB = 10*rand(10, 5);
%
%  % Compute the overlap ratio between every pair.
%  overlapRatio = bboxOverlapRatio(bboxA, bboxB)
%
%  See also selectStrongestBbox, selectStrongestBboxMulticlass.

%  Copyright 2013-2023 The MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

narginchk(2,3);
[bboxA, bboxB, ratioType, isUsingCodeGeneration] = iParseInputs(bboxA,bboxB,varargin{:});

if (isa(bboxA,'double') || isa(bboxB,'double'))
    bboxA = double(bboxA);
    bboxB = double(bboxB);
else
    bboxA = single(bboxA);
    bboxB = single(bboxB);
end

if (isempty(bboxA) || isempty(bboxB))
    overlapRatio = zeros(size(bboxA, 1), size(bboxB, 1), 'like', bboxA);
    return;
end

% The width of the input determines the type of box.
switch size(bboxA,2)
    case 4
        % axis-aligned rectangle
        overlapRatio = iOverlapRatioAxisAligned(bboxA, bboxB, ratioType, isUsingCodeGeneration);

    case 5
        % rotated rectangle
        overlapRatio = iOverlapRatioRotatedRect(bboxA, bboxB, ratioType, isUsingCodeGeneration);

    otherwise
        % Define all execution paths for codegen support. This codepath is
        % never executed because of runtime checks that ensure size(bboxA)
        % is 4.
        overlapRatio = zeros(size(bboxA, 1), size(bboxB, 1), 'like', bboxA);
end
end

%--------------------------------------------------------------------------
function [bboxA, bboxB, ratioType, isCodegen] = iParseInputs(bboxA, bboxB, varargin)
isCodegen = ~isempty(coder.target);
if isCodegen
    [bboxA, bboxB, ratioType] = validateAndParseInputsCodegen(bboxA, bboxB, varargin{:});
else
    [bboxA, bboxB, ratioType] = validateAndParseInputs(bboxA, bboxB, varargin{:});
end
checkSameBoxFormat(bboxA, bboxB);
end

%--------------------------------------------------------------------------
function overlapRatio = iOverlapRatioAxisAligned(bboxA, bboxB, ratioType, isCodegen)

if isCodegen
    overlapRatio = bboxOverlapRatioCodegen(bboxA, bboxB, ratioType);
else
    if strncmpi(ratioType, 'Union', 1)
        overlapRatio = visionBboxIntersectByUnion(bboxA, bboxB);
    else
        overlapRatio = visionBboxIntersectByMin(bboxA, bboxB);
    end
end
end

%--------------------------------------------------------------------------
function iou = iOverlapRatioRotatedRect(bboxA, bboxB, ratioType, isCodegen)

if isCodegen
    iou = bboxOverlapRatioRotatedRectCodegen(bboxA, bboxB, ratioType);
else
    % Convert to box vertices.
    [xa,ya] = vision.internal.bbox.bbox2poly(bboxA);
    [xb,yb] = vision.internal.bbox.bbox2poly(bboxB);

    if strncmpi(ratioType, 'Union', 1)
        iou = visionRotatedBBoxIntersectByUnion(xa,ya,xb,yb);
    else
        iou = visionRotatedBBoxIntersectByMin(xa,ya,xb,yb);
    end
end
end

%--------------------------------------------------------------------------
function checkSameBoxFormat(bboxA,bboxB)
if ~isempty(bboxA) && ~isempty(bboxB)
    coder.internal.errorIf(size(bboxA,2) ~= size(bboxB,2), ...
        'vision:bbox:boxFormatMustMatch');
end
end

%--------------------------------------------------------------------------
function checkInputBoxes(bbox)
% Validate the input boxes

% Numeric objects are not supported.
vision.internal.inputValidation.validateNotObject(bbox,'vision','bbox');

validateattributes(bbox,{'uint8', 'int8', 'uint16', 'int16', 'uint32', ...
    'int32', 'single', 'double'}, {'real','nonsparse','finite','2d'}, ...
    mfilename);

if ~isempty(bbox)
    coder.internal.errorIf(~any(size(bbox,2) == [4 5]), ...
        'vision:bbox:invalidBoxFormat');
    coder.internal.errorIf((any(bbox(:,3)<=0) || any(bbox(:,4)<=0)), ...
        'vision:visionlib:invalidBboxHeightWidth');
end
end

%--------------------------------------------------------------------------
function checkRatioType(value)
% Validate the input ratioType string

list = {'Union', 'Min'};
validateattributes(value, {'char','string'}, {'nonempty'}, mfilename, 'RatioType');

validatestring(value, list, mfilename, 'RatioType');
end

%--------------------------------------------------------------------------
function [bboxA, bboxB, ratioType] = validateAndParseInputs(bboxA,bboxB,varargin)
% Validate and parse optional inputs

% Setup parser
parser = inputParser;
parser.CaseSensitive = false;
parser.FunctionName  = mfilename;

parser.addRequired('bboxA', @checkInputBoxes);
parser.addRequired('bboxB', @checkInputBoxes);
parser.addOptional('RatioType', 'Union', @checkRatioType);

% Parse input
parser.parse(bboxA,bboxB,varargin{:});

bboxA = parser.Results.bboxA;
bboxB = parser.Results.bboxB;
ratioType = char(parser.Results.RatioType);

end

%--------------------------------------------------------------------------
function [bboxA, bboxB, ratioType] = validateAndParseInputsCodegen(bboxA,bboxB,varargin)
% Validate and parse optional inputs

eml_lib_assert(nargin >= 2, 'vision:visionlib:NotEnoughArgs', 'Not enough input arguments');
eml_lib_assert(nargin <= 3, 'vision:visionlib:TooManyArgs', 'Too many input arguments.');

checkInputBoxes(bboxA);
checkInputBoxes(bboxB);

if nargin == 3
    ratioType = varargin{1};
    validateattributes(ratioType, {'char'}, {'nonempty'}, mfilename, 'ratioType');
    validatestring(ratioType, {'Union', 'Min'}, mfilename, 'ratioType');
else
    ratioType = 'Union';
end

end

%--------------------------------------------------------------------------
function overlapRatio = bboxOverlapRatioCodegen(bboxA, bboxB, ratioType)
% Compute the overlap ratio between every row in bboxA and bboxB

% left top corner
x1BboxA = bboxA(:, 1);
y1BboxA = bboxA(:, 2);
% right bottom corner
x2BboxA = x1BboxA + bboxA(:, 3);
y2BboxA = y1BboxA + bboxA(:, 4);

x1BboxB = bboxB(:, 1);
y1BboxB = bboxB(:, 2);
x2BboxB = x1BboxB + bboxB(:, 3);
y2BboxB = y1BboxB + bboxB(:, 4);

% area of the bounding box
areaA = bboxA(:, 3) .* bboxA(:, 4);
areaB = bboxB(:, 3) .* bboxB(:, 4);

overlapRatio = zeros(size(bboxA,1),size(bboxB,1), 'like', bboxA);

for m = 1:size(bboxA,1)
    for n = 1:size(bboxB,1)
        % compute the corners of the intersect
        x1 = max(x1BboxA(m), x1BboxB(n));
        y1 = max(y1BboxA(m), y1BboxB(n));
        x2 = min(x2BboxA(m), x2BboxB(n));
        y2 = min(y2BboxA(m), y2BboxB(n));

        % skip if there is no intersection
        w = x2 - x1;
        if w <= 0
            continue;
        end

        h = y2 - y1;
        if h <= 0
            continue;
        end

        intersectAB = w * h;
        if strncmpi(ratioType, 'Union', 1) % divide by union of bboxA and bboxB
            overlapRatio(m,n) = intersectAB/(areaA(m)+areaB(n)-intersectAB);
        else % divide by minimum of bboxA and bboxB
            overlapRatio(m,n) = intersectAB/min(areaA(m), areaB(n));
        end
    end
end

end

%--------------------------------------------------------------------------
function overlapRatio = bboxOverlapRatioRotatedRectCodegen(bboxA, bboxB, ratioType)
% Compute the overlap ratio between every row in bboxA and bboxB

[xa, ya] = vision.internal.bbox.bbox2poly(bboxA);
[xb, yb] = vision.internal.bbox.bbox2poly(bboxB);

% area of the bounding box
areaA = polyarea(xa, ya);
areaB = polyarea(xb, yb);

overlapRatio = zeros(size(bboxA,1),size(bboxB,1), 'like', bboxA);

for m = 1:size(bboxA,1)
    for n = 1:size(bboxB,1)
        % Get the polygon1 points
        xIndices1 = [xa(:,m);xa(1,m)];
        yIndices1 = [ya(:,m);ya(1,m)];
        xIndices2 = [xb(:,n);xb(1,n)];
        yIndices2 = [yb(:,n);yb(1,n)];

        % Get the overlap polygon points
        overlapedPolyPts = vision.internal.detector.overlapPolygonRotatedRects. ...
            findOverlapPolyPoints(xIndices1, yIndices1, xIndices2, yIndices2);
        % skip if there is no intersection
        if isempty(overlapedPolyPts)
            continue;
        end

        % Get the overlap polygon
        overlapPoly = vision.internal.detector.overlapPolygonRotatedRects. ...
            findOverlapPolygon(overlapedPolyPts);
        intersectAB = polyarea(overlapPoly(1,:),overlapPoly(2,:));

        if strncmpi(ratioType, 'Union', 1) % divide by union of bboxA and bboxB
            overlapRatio(m,n) = intersectAB/(areaA(m)+areaB(n)-intersectAB);
        else % divide by minimum of bboxA and bboxB
            overlapRatio(m,n) = intersectAB/min(areaA(m), areaB(n));
        end
    end
end

end
