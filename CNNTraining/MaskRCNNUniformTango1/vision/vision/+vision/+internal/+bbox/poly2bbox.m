function bbox = poly2bbox(X,Y)
% bbox = poly2bbox(X,Y) returns an axis-aligned bounding box from a set of
% polygon vertices in X and Y. X and Y are N-by-M matrices representing M
% polygons with N vertices. X and Y are in spatial coordinates. bbox is
% returned in spatial coordinates in the min/max box format.
%
% This function is for internal use only and is likely to change in future
% releases.

% Copyright 2019 The MathWorks, Inc.

xmin = min(X)';
xmax = max(X)';
ymin = min(Y)';
ymax = max(Y)';

bbox = [xmin ymin xmax ymax];

