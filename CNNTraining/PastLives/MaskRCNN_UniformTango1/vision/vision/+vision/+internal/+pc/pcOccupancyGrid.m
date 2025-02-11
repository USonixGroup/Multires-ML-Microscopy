function occupancyGrid = pcOccupancyGrid(ptCloud, gridSize, gridStep,...
                                zLimit)
%pcOccupancy Create 2-d occupancy grid of a point cloud
%
%   occupancyGrid = pcOccupancyGrid(ptCloudIn, gridSize, gridStep,
%   zLimit) returns a 2-d occupancy grid of the ptCloudIn point cloud
%   object. ptCloudIn is an unorganized point cloud. gridSize is a scalar
%   value in world units specifying the size of the occupancy grid. The
%   width and the height of the occupancy grid span from -gridSize/2 to
%   gridSize/2. gridStep is a scalar value specifying the size of
%   each grid in world units. zLimit is a 1-by-2 vector specifying the
%   limits for assigning the probabilities to the grids.
%
  
% Copyright 2020 The Mathworks, Inc.
%
% References:
% -----------
% M. Dimitrievski, D. Van Hamme, P. Veelaert and W. Philips, "Robust
% matching of occupancy maps for odometry in autonomous vehicles", (2016)
% Proceedings of 2016 VISAPP. p.626-633

%#codegen

xyzPoints = ptCloud.Location;
numPoints = ptCloud.Count;

spatialLimits = [-gridSize/2 gridSize/2; -gridSize/2 gridSize/2];
zMin = zLimit(1);
zMax = zLimit(2);

% Calclate the number of bins
gridXBinSize = round(abs(spatialLimits(1,2) - spatialLimits(1,1)) / gridStep);
gridYBinSize = round(abs(spatialLimits(2,2) - spatialLimits(2,1)) / gridStep);

occupancyGrid = zeros(gridXBinSize, gridYBinSize, 'like', xyzPoints);
gridCount = zeros(gridXBinSize, gridYBinSize);

% Calculate the edges of the bins
xEdges = linspace(spatialLimits(1,1),spatialLimits(1,2),gridXBinSize+1);
yEdges = linspace(spatialLimits(2,1),spatialLimits(2,2),gridYBinSize+1);

% Calculate x-y indices of the points in point cloud
xIndices = discretize(xyzPoints(:,2), xEdges);
yIndices = discretize(xyzPoints(:,1), yEdges);

zValues = xyzPoints(:,3);

% Assign a probabilty scaling from 0 to 1 for z values [zMin zMax].
% For values greater than zMax assign 1 and value less than zMin
% assign 0.
% Should be [0 - 1]
zValues = rescale(zValues, 'InputMin', zMin, 'InputMax', zMax);

for i = 1:numPoints
    
    xIdx = xIndices(i);
    yIdx = yIndices(i);
    
    if ~isnan(xIdx) && ~isnan(yIdx)

        occupancyGrid(xIdx, yIdx) = occupancyGrid(xIdx, yIdx) + zValues(i);

        gridCount(xIdx, yIdx) = gridCount(xIdx, yIdx) + 1;
    end
end

gridCount(gridCount == 0) = 1;

occupancyGrid = occupancyGrid ./ gridCount;
end