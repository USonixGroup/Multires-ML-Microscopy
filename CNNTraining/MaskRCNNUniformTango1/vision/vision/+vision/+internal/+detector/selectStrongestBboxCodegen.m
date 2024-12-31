function isKept = selectStrongestBboxCodegen(...
    inputBbox, overlapThreshold, isDivByUnion, numStrongest,...
    isRotatedRectangles, varargin)
% Codegen implementation for selectStrongestBbox and
% selectStrongestBboxMulticlass.

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen

if numel(varargin) == 1
    inputLabel = varargin{1};
    hasLabel = true;
else
    inputLabel = [];
    hasLabel = false;
end

isKept = true(size(inputBbox,1), 1);

if ~isRotatedRectangles
    area = inputBbox(:,3).*inputBbox(:,4);
    x1 = inputBbox(:,1);
    x2 = inputBbox(:,1)+inputBbox(:,3);
    y1 = inputBbox(:,2);
    y2 = inputBbox(:,2)+inputBbox(:,4);
else
    [x, y] = vision.internal.bbox.bbox2poly(inputBbox);
    area = polyarea(x, y);
end
% Determine the loop management method based on numStrongest.
loopManagementMethod = iDetermineLoopManagementMethod(numStrongest);

% Initialize box counts based on numStrongest.
boxCounts = zeros(1, numel(numStrongest));

% For each bbox i, suppress all surrounded bbox j where j>i and overlap
% ratio is larger than overlapThreshold
numOfBbox = size(inputBbox,1);
currentBox = 0;
label = zeros(1,1,'like',inputLabel);
for i = 1:numOfBbox

    % Keep track of current index in case loop breaks.
    currentBox = i;

    if isRotatedRectangles
        % Get the polygon1 points
        xIndices1 = x(:,i);
        yIndices1 = y(:,i);
    end

    if hasLabel
        label = inputLabel(i);

        if isnan(label)
            % Do not select boxes w/ undefined labels.
            isKept(i) = false;
            continue;
        end
    end

    if loopManagementMethod == 1
        % Find all.
        status = iDetermineLoopStatusFindAll(isKept(i));

    elseif loopManagementMethod == 2
        % Find top-k boxes across classes.
        [status, boxCounts] = iDetermineLoopStatusTopK(numStrongest, i, boxCounts, isKept);

    else
        % Find top-k boxes per class.
        [status, boxCounts, isKept] = iDetermineLoopStatusTopKPerClass(numStrongest, i, boxCounts, isKept, label);
    end

    if iShouldBreak(status)
        % Terminate loop.
        break

    elseif iShouldContinue(status)
        % Continue to next box.
        continue
    else
        % Scan through lower scoring boxes and suppress overlapping boxes.
        parfor j = (i+1):numOfBbox
            if ~isKept(j)
                continue;
            end

            if hasLabel && inputLabel(j) ~= label
                continue
            end

            % Compute the intersect box
            if ~isRotatedRectangles
                width = min(x2(i), x2(j)) - max(x1(i), x1(j));
                if width <= 0
                    continue;
                end

                height = min(y2(i), y2(j)) - max(y1(i), y1(j));
                if height <= 0
                    continue;
                end

                areaOfIntersect = width * height;

            else

                % Get the polygon 2 points
                xIndices2 = x(:,j);
                yIndices2 = y(:,j);

                % Check whether two bounding boxes overlap or not
                if axisBoundsDoNotOverlap(xIndices1, yIndices1, xIndices2, yIndices2)
                    continue;
                end

                % Get the overlap polygon points
                overlapedPolyPts = vision.internal.detector.overlapPolygonRotatedRects.findOverlapPolyPoints(xIndices1, yIndices1, xIndices2, yIndices2);

                % Skip if there is no intersection
                if isempty(overlapedPolyPts)
                    continue;
                end

                % Get the overlapped polygon
                overlapPoly = vision.internal.detector.overlapPolygonRotatedRects.findOverlapPolygon(overlapedPolyPts);
                areaOfIntersect = polyarea(overlapPoly(1,:),overlapPoly(2,:));
            end

            if isDivByUnion
                overlapRatio = areaOfIntersect/(area(i)+area(j)-areaOfIntersect);
            else
                overlapRatio = areaOfIntersect/min(area(i), area(j));
            end

            if overlapRatio > overlapThreshold
                isKept(j) = false;
            end
        end
    end

end

% When the number of strongest boxes is reached, set the remainder to
% false.
isKept(currentBox+1:end) = false;
end

%--------------------------------------------------------------------------
function method = iDetermineLoopManagementMethod(numStrongest)
% Return the method used to determine the loop termination status based on
% number of strongest bounding boxes.
%
%   Method 1: find all
%   Method 2: top-k across all boxes
%   Method 3: top-k per class

if numel(numStrongest) == 1
    if isinf(numStrongest)
        method = 1;
    else
        method = 2;
    end
else
    method = 3;
end
end

%--------------------------------------------------------------------------
function status = iDetermineLoopStatusFindAll(isKept)
% Return loop status of continue when isKept is false, otherwise return
% the resume status.
if isKept
    % Keep searching for boxes.
    status = iLoopStatusResume();
else
    % This box is already suppressed. Continue to next box.
    status = iLoopStatusContinue();
end
end

%--------------------------------------------------------------------------
function [status, boxCount] = iDetermineLoopStatusTopK(...
    numStrongest, i, boxCount, isKept)
% Return loop status and updated boxCount. When the box count reaches
% numStrongest, the loop status is break. Otherwise, it is resume or
% continue based on the value of isKept(i).

if isKept(i)
    if boxCount < numStrongest
        boxCount = boxCount + 1;
    end

    if boxCount == numStrongest
        status = iLoopStatusBreak();
    else
        status = iLoopStatusResume();
    end
else
    status = iLoopStatusContinue();
end

end

%--------------------------------------------------------------------------
function [status, boxCounts, isKept] = iDetermineLoopStatusTopKPerClass(...
    numStrongest, i, boxCounts, isKept, label)
% Return loop status and updated boxCounts and isKept values. When all
% classes reach the their respective number of strongest boxes, the loop
% status is break. When the current box is already at the max limit, the
% loop status is continue. Otherwise, loop status is resume.

if isKept(i)

    % Increment the box count for this class if the count is less
    % than the maximum number of strongest boxes.
    justReachedLimit = false;
    if boxCounts(label) < numStrongest(label)
        boxCounts(label) = boxCounts(label) + 1;
        justReachedLimit = boxCounts(label) == numStrongest(label);
    end

    % Check whether we have enough boxes for all classes. When we
    % reach the numStrongest limit, break out of the search.
    % Otherwise, if we already have enough boxes for the
    % current label and continue to the next box. If neither of
    % these cases are true, keep scanning the boxes.

    if all(boxCounts(:) == numStrongest(:))
        % Terminate loop.
        status = iLoopStatusBreak();
    elseif boxCounts(label) == numStrongest(label)
        % Already have the max number of current class. Do not keep any
        % more instances of this class unless we've just reached the
        % limit.
        isKept(i) = justReachedLimit;

        % Continue to next box.
        status = iLoopStatusContinue();
    else
        % Keep searching.
        status = iLoopStatusResume();
    end

else
    % Already suppressed. Continue to next box.
    status = iLoopStatusContinue();
end
end

%------------------------------------------------------------------------
function out = axisBoundsDoNotOverlap(xa, ya, xb, yb)
abounds = axisbounds(xa, ya);
bbounds = axisbounds(xb, yb);

xmin = max(abounds(1), bbounds(1));
ymin = max(abounds(2), bbounds(2));
xmax = min(abounds(3), bbounds(3));
ymax = min(abounds(4), bbounds(4));

out = ((xmax-xmin <= 0) || (ymax-ymin <= 0));
end

%--------------------------------------------------------------------------
function out = axisbounds(x, y)
xmin = min(x(1:4,1));
xmax = max(x(1:4,1));
ymin = min(y(1:4,1));
ymax = max(y(1:4,1));
out = [xmin, ymin, xmax, ymax];
end
%--------------------------------------------------------------------------
function status = iLoopStatusBreak()
coder.inline('always');
status = 0;
end

%--------------------------------------------------------------------------
function status = iLoopStatusContinue()
coder.inline('always');
status = 1;
end

%--------------------------------------------------------------------------
function status = iLoopStatusResume()
coder.inline('always');
status = 2;
end

%--------------------------------------------------------------------------
function tf = iShouldBreak(status)
coder.inline('always');
tf = status == iLoopStatusBreak();
end

%--------------------------------------------------------------------------
function tf = iShouldContinue(status)
coder.inline('always');
tf = status == iLoopStatusContinue();
end
