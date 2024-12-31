function vertices = rotatedRectangleHeadDisplay(I, position, lineWidth)
% vertices = rotatedRectangleHeadDisplay(I, position, lineWidth) returns
% seven vertices that are used to draw an orientation arrow annotation that
% denotes the "head" of a rotated rectangle. vertices is an M-by-14 matrix
% of x,y vertice locations, where M is the number of rotated rectangles being
% annotated and each row is represented as [vert1X, vert1Y, ..., vert7X, vert7Y].
% The vertices are as follows:
% ----------------------------------
% -------------------vert5----------
% ---------------------|---\--------
% ---------------------|----\-------
% -----vert7_________vert6---\------
% -------|--------------------\-----
% -------|-------------------vert4--
% -------|--------------------/-----
% -----vert1_________vert2---/------
% ---------------------|----/-------
% ---------------------|---/--------
% -------------------vert3----------
% ----------------------------------

% Copyright 2023 The MathWorks, Inc.

%#codegen

% Convert the rotated rectangle to vertices
[X, Y] = vision.internal.bbox.bbox2poly(double(position));

% Convert the orientation angle to between 0 and 360 degrees
angle = double(mod(position(:,5),360));

% Find the sign of each angle and assume + for angle == 0 deg.
angleSign = sign(angle);
angleSign(angleSign == 0) = 1;

% Declare header arrow size and find the arrow's tip vertice so that the
% arrow's stem may scale down near the image's borders. The arrow will scale
% based off the size of the rotated rectangle and the lineWidth input.
headSize = hypot(X(2,:)-X(3,:),Y(2,:)-Y(3,:)); % Length of rectangle head side
headCenterPoint = [(X(2,:) + X(3,:))./2; (Y(2,:) + Y(3,:))./2]'; % Midpoint of head

baseLengthRatio = max(5,0.10*headSize'); % Ratio of the stem length to the rectangle head side
baseWidthRatio = max(2,0.02*headSize'); % Ratio of the stem width to the rectangle head side
baseLength = (1 + 0.15*lineWidth)*baseLengthRatio; % Length of the arrow stem
baseWidth = (1 + 0.15*lineWidth)*baseWidthRatio; % Width of the arrow stem

% Change in X and Y from stem base to stem head
xDelta = cosd(angle).*baseLength;
yDelta = sind(angle).*baseLength;

% Change in X and Y from head center stem base vertices
baseXDelta = cosd(angle+angleSign.*90).*(baseWidth/2);
baseYDelta = sind(angle+angleSign.*90).*(baseWidth/2);

% Change in X and Y from arrow base center
triXDelta = cosd(angle+angleSign.*90).*(baseWidth*2);
triYDelta = sind(angle+angleSign.*90).*(baseWidth*2);

% Calculate the arrow's tip vertice
triBaseCenter = headCenterPoint + [xDelta, yDelta];
vert4 = triBaseCenter + 0.5*[xDelta, yDelta];
vert4Clamped = zeros(size(vert4));
vert4Clamped(:,1) = max(0,min(vert4(:,1),size(I,2)));
vert4Clamped(:,2) = max(0,min(vert4(:,2),size(I,1)));

% Calculate the clamp delta for the header arrow
vert4Diff = vert4-vert4Clamped;

clampXDelta = tand(90-angle).*abs(vert4Diff(:,2));
clampYDelta = tand(angle).*abs(vert4Diff(:,1));
clampXFixIdx = clampXDelta == 0 | isnan(clampXDelta) | isinf(clampXDelta);
clampYFixIdx = clampYDelta == 0 | isnan(clampYDelta) | isinf(clampYDelta);
clampXDelta(clampXFixIdx) = vert4Diff(clampXFixIdx,1);
clampYDelta(clampYFixIdx) = vert4Diff(clampYFixIdx,2);
clampLinearDist = hypot(clampXDelta,clampYDelta);

% Readjust vert4, the arrow point
vert4 = vert4 - [clampXDelta, clampYDelta];
 
% Recalculate X and Y deltas from stem base to stem head and triBaseCenter
baseLength = abs((1 + 0.1*lineWidth)*baseLengthRatio-clampLinearDist);
xDelta = cosd(angle).*baseLength;
yDelta = sind(angle).*baseLength;
triBaseCenter = headCenterPoint + [xDelta, yDelta];

% Snap arrow head base to rotated rectangle head if it has entered the
% rotated rectangle's interior. Also set xDelta and yDelta to zero for
% affected rotated rectangle heads.
triBaseCenterDiff = triBaseCenter - double(position(:,1:2));
triBaseCenterDist = hypot(triBaseCenterDiff(:,1),triBaseCenterDiff(:,2));
headBaseCenterDiff = headCenterPoint - double(position(:,1:2));
headBaseCenterDist = hypot(headBaseCenterDiff(:,1), headBaseCenterDiff(:,2));
triInteriorIdx = triBaseCenterDist < headBaseCenterDist;

triBaseCenter(triInteriorIdx,:) = headCenterPoint(triInteriorIdx,:);
xDelta(triInteriorIdx) = 0;
yDelta(triInteriorIdx) = 0;

% Find the base vertices of the arrow header. The arrow (triangle) base
% is to be 4x the base width.
vert1 = headCenterPoint + [baseXDelta, baseYDelta];
vert2 = vert1 + [xDelta, yDelta];
vert7 = headCenterPoint - [baseXDelta, baseYDelta];
vert6 = vert7 + [xDelta, yDelta];

% Find and assign the vertices of the arrow header's head (triangle)
vert3 = triBaseCenter + [triXDelta, triYDelta];
vert5 = triBaseCenter - [triXDelta, triYDelta];

vertices= [vert1 vert2 vert3 vert4 vert5 vert6 vert7];

end