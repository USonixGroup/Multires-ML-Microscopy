classdef overlapPolygonRotatedRects
    % Shared helper functions for finding overlap polygon formed from
    % rotated rectangles for selectStrongestBboxCodegen and bboxOverlapRatio.

    % Copyright 2021-2023 The MathWorks, Inc.
    %
    % References
    % ----------
    % [1] https://developer.nvidia.com/blog/detecting-rotated-objects-using-the-odtk/
    %
    % [2] Stephen I. Warshaw, M. (1977). Area of Intersection of Arbitrary Polygons.
    %
    % [3] https://geomalgorithms.com/a13-_intersect-4.html

    %#codegen
    %#ok<*EMCLS>
    %#ok<*EMCA>
    methods(Static)

        %------------------------------------------------------------------
        % findOverlapPolyPoints  Computes the overlap points formed from
        %                        two polygons.
        % Inputs
        %   xIndices1    - xIndices of polygon1.
        %   yIndices1    - yIndices of polygon1.
        %   xIndices2    - xIndices of polygon2.
        %   yIndices2    - yIndices of polygon2..
        %
        % Output
        %   overlapedPolyPts - overlap Polygon Points.
        %------------------------------------------------------------------
        function overlapedPolyPts = findOverlapPolyPoints(xIndices1, yIndices1, xIndices2, yIndices2)
            coder.inline('never')
            % Each line segment of a rectangle can have maximum of two intersection
            % points on another rectangle, the maximum number of intersection points
            % between two overlapping rotated rectangles is 8.
            % Initialize the variable with predefined size to store the
            % intersection points, varsize is not used due to performance
            % issues.
            overlapedPolyPts = zeros(2,8);
            count = 0;
            coder.unroll(true);
            for i = 1:4
                next = mod(i,4) + 1;
                polygon1Point1 = [xIndices1(i) yIndices1(i)];
                polygon1Point2 = [xIndices1(next) yIndices1(next)];
                % Line p1p2 represented as a1x + b1y = c1
                a1 = polygon1Point2(2) - polygon1Point1(2);
                b1 = polygon1Point1(1) - polygon1Point2(1);
                c1 = a1*polygon1Point1(1) + b1*polygon1Point1(2);
                coder.unroll(true);
                for j = 1:4
                    nextb = mod(j,4) + 1;
                    polygon2Point1 = [xIndices2(j) yIndices2(j)];
                    polygon2Point2 = [xIndices2(nextb) yIndices2(nextb)];
                    % Line q1q2 represented as a2x + b2y = c2
                    a2 = polygon2Point2(2) - polygon2Point1(2);
                    b2 = polygon2Point1(1) - polygon2Point2(1);
                    c2 = a2*polygon2Point1(1) + b2*polygon2Point1(2);

                    det = a1*b2 - a2*b1;
                    if (det==0)
                        continue;
                    end
                    insX = (b2*c1 - b1*c2)/det;
                    insY = (a1*c2 - a2*c1)/det;
                    intersectPt = [insX; insY];
                    % Check if the intersection point lies on the line segment
                    if(isPointOnLine(intersectPt, polygon1Point1, polygon1Point2) && isPointOnLine(intersectPt, polygon2Point1, polygon2Point2))
                        count = count + 1;
                        overlapedPolyPts(:,count) = intersectPt;
                    end
                end
            end
            % Points located inside or on edge of polygon
            pts1 = insidePoly(xIndices1, yIndices1, xIndices2, yIndices2);
            pts2 = insidePoly(xIndices2, yIndices2, xIndices1, yIndices1);
            % Final coordinates of the overlapping polygon
            overlapedPolyPts = [overlapedPolyPts(:,1:count), pts1, pts2];
        end

        %------------------------------------------------------------------
        % findOverlapPolygon  Computes the overlap ordered polygon points from
        %                     unordered points.
        %
        % Inputs
        %   overlapedPolyPts  - unordered polygon points.
        %
        % Output
        %   overlapPoly - ordered polygon Points.
        %------------------------------------------------------------------
        function overlapPoly = findOverlapPolygon(overlapedPolyPts)
            coder.inline('never')
            % Center of poly points
            center = mean(overlapedPolyPts, 2);
            % Vectors connecting the center point and the given points
            distVec = overlapedPolyPts - center;
            % Finding the angles above x axis
            theta = atan2(distVec(2,:),distVec(1,:));
            % Sorting the angles to find the aligned vertices
            [~, idx] = sort(theta);
            % Sorting the given points to form the polygon
            overlapedPolyPts = overlapedPolyPts(:,idx);
            % Add the first vertex at the end to close the polygon
            overlapPoly = [overlapedPolyPts overlapedPolyPts(:,1)];
        end

    end

end

%------------------------------------------------------------------
function out = isPointOnLine(p, p1, p2)
% Epsilon value is chosen as checking the vertices values upto
% 3 decimals is enough.
epsilon = 0.001;
if ((p(1) - epsilon <= max(p1(1),p2(1))) && ...
        (p(1) + epsilon >= min(p1(1),p2(1))) && ...
        (p(2) - epsilon <= max(p1(2),p2(2))) && ...
        (p(2) + epsilon >= min(p1(2),p2(2))) )
    out = true;
else
    out = false;
end
end

%--------------------------------------------------------------------------
function points = insidePoly(xa, ya, xb, yb)
% The maximum number of points of a rectangle which can be inside another
% rectangle are 8 when one rectangle exactly overlaps with another rectangle.
% Initialize the variable with predefined size to store the points present
% inside the rectangle, varsize is not used due to performance issues.

pointsInsidePoly = zeros(2,8);
count = 0;
% Looping over edges of polygon (xa,ya)
coder.unroll(true);
for i = 1:4
    polygon1Point = [xa(i), ya(i)];
    neg = 0;
    pos = 0;
    % Looping over edges of polygon (xb,yb)
    coder.unroll(true);
    for j = 1:4
        next = mod(j,4) + 1;
        polygon2Point1 = [xb(j), yb(j)];
        polygon2Point2 = [xb(next), yb(next)];
        cross = ((polygon1Point(2) - polygon2Point1(2)) * (polygon2Point2(1) - polygon2Point1(1))) - ((polygon1Point(1) - polygon2Point1(1)) * (polygon2Point2(2) - polygon2Point1(2)));
        if cross == 0
            if isPointOnLine(polygon1Point, polygon2Point1, polygon2Point2)
                count = count + 1;
                pointsInsidePoly(:,count) = polygon1Point';
            end
        elseif cross < 0
            neg = neg + 1;
        elseif cross > 0
            pos = pos + 1;
        end
    end
    if(neg/4==1 || pos/4==1)
        count = count + 1;
        pointsInsidePoly(:,count) = polygon1Point';
    end
end
points = pointsInsidePoly(:,1:count);
end

