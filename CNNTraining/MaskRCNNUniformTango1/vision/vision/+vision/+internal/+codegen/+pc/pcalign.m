function [ptCloudOut] = pcalign(pc, tformsMat, voxelSize)
%
%   This function is used to implement code generation support for the
%   pcalign function.

% Copyright 2021 The MathWorks, Inc.
%#codegen

validateattributes(pc, {'pointCloud'}, {'vector', 'nonempty'}, ...
    'pcalign', 'pc');

validateattributes(tformsMat, {'rigidtform3d','affinetform3d','rigid3d','affine3d'}, ...
    {'vector', 'numel', numel(pc)}, 'pcalign', 'tforms');

[validClouds, validIndices, hasColor, hasNormal, hasIntensity] = ...
    findValidPtclouds(pc);

if validClouds
    [ptCloud, tforms] = extractPointClouds(pc, tformsMat, validClouds, validIndices);
    
    numClouds = numel(ptCloud);
    
    [tformedClouds, sizeArray, limits, totalNumPoints]= transformPtCloud(ptCloud, tforms, hasColor,...
        hasNormal, hasIntensity, numClouds);
    
    isVoxelGridFilterNeeded = ~isempty(voxelSize) && totalNumPoints > 0;
    if (isVoxelGridFilterNeeded)
        
        [sortIndexVecIdx, total] = computeVoxelGridIndex(tformedClouds, limits, ...
            totalNumPoints, voxelSize);
        [filteredLocation, filteredColor, filteredIntensity, filteredNormal] = ...
            vision.internal.buildable.pcalignBuildable.voxelGridFilter(tformedClouds,...
            total, sortIndexVecIdx,...
            hasColor, hasNormal, hasIntensity, sizeArray);
        
        %     Make point cloud
        ptCloudOut = pointCloud(filteredLocation, 'Color', filteredColor, 'Normal',...
            filteredNormal, 'Intensity', filteredIntensity);
    else
        ptCloudOut = vision.internal.codegen.pc.pccat(tformedClouds);
    end
else
    locationType = class(pc(1).Location);
    colorType = class(pc(1).Color);
    intensityType = class(pc(1).Intensity);
    ptCloudOut = pointCloud(zeros(0,3, locationType), 'Color', zeros(0,3,colorType),...
        'Normal', zeros(0,3,locationType), 'Intensity', zeros(0,1,intensityType));
end
end

function [validClouds, validIndices, hasColor, hasNormal, hasIntensity] = ...
    findValidPtclouds(pc)

numClouds = numel(pc);
validIdx  = zeros(numClouds, 1);

hasColor     = true;
hasIntensity = true;
hasNormal    = true;

for n =1:numClouds
    if ~isempty(pc(n).Location)
        tf = isfinite(pc(n).Location);
        if ismatrix(tf)
            indices = (sum(tf, 2) == 3);
        else
            indices = (sum(reshape(tf, [], 3), 2) == 3);
        end
        if(sum(indices(:))>0)
            validIdx(n)  = 1;
            hasColor     = hasColor && ~isempty(pc(n).Color);
            hasIntensity = hasIntensity && ~isempty(pc(n).Intensity);
            hasNormal    = hasNormal && ~isempty(pc(n).Normal);
        end
    end
end

validIndices = find(validIdx == 1);
validClouds = size(validIndices, 1);
end
%//////////////////////////////////////////////////////////////////////////
% Extract valid point clouds and tforms
%//////////////////////////////////////////////////////////////////////////
function [ptCloud, tforms] = ...
    extractPointClouds(pc, tformsMat, validClouds, validIndices)

tforms      = zeros(4, 4, validClouds, 'like', tformsMat(1).T);

ptCloud = pointCloud(pc(validIndices(1)).Location, 'Color', pc(validIndices(1)).Color,...
    'Normal', pc(validIndices(1)).Normal, 'Intensity',  pc(validIndices(1)).Intensity);

if(validClouds > 1)
    ptCloud(2:validClouds) = pc(validIndices(2:validClouds));
end

for n = 1:validClouds
    tforms(:,:,n) = tformsMat(validIndices(n)).T;
end
end
%//////////////////////////////////////////////////////////////////////////
%// Transform point clouds, removing invalid points
%////////////////////////////////////////////////////////////////////////
function [tformedClouds, sizeArray, limits, totalNumPoints] = transformPtCloud(ptCloudArr , tformsMat, hasColor,...
    hasNormal, hasIntensity, numClouds)

tform       = tformsMat(:,:,1);
[location, color, normal, intensity] = extractValidPoints(ptCloudArr(1));

points      = [location, ones(size(location,1), 1)];
tformPoints = points*tform(1:4,1:3);

sizeArray    = zeros(numClouds, 1, 'uint32');
sizeArray(1) = uint32(size(location, 1));

if ~hasColor
    color = cast(zeros(0,0), 'like', ptCloudArr(1).Color);
end

if hasNormal
    normalVec   = [normal, zeros(size(location,1), 1)];
    tformNormal = normalVec*tform(1:4,1:3);
else
    tformNormal = cast(zeros(0,0), 'like', location);
end

if ~hasIntensity
    intensity = cast([], 'like', ptCloudArr(1).Intensity);
end

tformedClouds = pointCloud(tformPoints, 'Color', color, 'Normal', tformNormal,...
    'Intensity', intensity);

limits = [Inf, -Inf, Inf, -Inf, Inf, -Inf];
totalNumPoints = 0;

limits         = findLimits(limits, tformedClouds, 1);
totalNumPoints = totalNumPoints + tformedClouds(1).Count;

if numClouds > 1
    for n = 2 : numClouds
        tform       = tformsMat(:,:,n);
        [location, color, normal, intensity] = extractValidPoints(ptCloudArr(n));
        
        sizeArray(n)  = uint32(size(location, 1));
        points        = [location, ones(size(location, 1), 1)];
        tformPoints   = points*tform(1:4,1:3);
        
        if ~hasColor
            color = cast(zeros(0,0), 'like', ptCloudArr(1).Color);
        end
        
        if hasNormal
            normalVec   = [normal, zeros(size(location, 1), 1)];
            tformNormal = normalVec*tform(1:4,1:3);
        else
            tformNormal = cast(zeros(0,0), 'like', location);
        end
        
        if ~hasIntensity
            intensity = cast([], 'like', ptCloudArr(1).Intensity);
        end
        
        tformedClouds(n) = pointCloud(tformPoints, 'Color', color, 'Normal', tformNormal,...
            'Intensity', intensity);
        
        limits         = findLimits(limits, tformedClouds, n);
        totalNumPoints = totalNumPoints + tformedClouds(n).Count;
        
    end
end

end
%//////////////////////////////////////////////////////////////////////////
% // Compute indexing vector for voxel grid filter
% ///////////////////////////////////////////////////////////////////////////
function [sortIndexVecIdx, total] = computeVoxelGridIndex(tformedPtCloudArr,...
    limits, totalNumPoints, voxelSize)

inverseVoxelSize = 1/voxelSize;
numPoints        = uint32(totalNumPoints);

xmin = limits(1);
xmax = limits(2);
ymin = limits(3);
ymax = limits(4);
zmin = limits(5);
zmax = limits(6);

dx = uint64(floor((xmax - xmin) * inverseVoxelSize) + 1);
dy = uint64(floor((ymax - ymin) * inverseVoxelSize) + 1);
dz = uint64(floor((zmax - zmin) * inverseVoxelSize) + 1);

dxy  = dx*dy;
dxyz = dxy*dz;

coder.internal.errorIf((dx == 0 || dy == 0 || dz == 0 || dxy == 0),...
    'vision:pointcloud:voxelSizeTooSmall');
coder.internal.errorIf((dx ~= dxy / dy || dz ~= dxyz / dxy),...
    'vision:pointcloud:voxelSizeTooSmall');

minbx = int64(floor(xmin * inverseVoxelSize));
maxbx = int64(floor(xmax * inverseVoxelSize));
minby = int64(floor(ymin * inverseVoxelSize));
maxby = int64(floor(ymax * inverseVoxelSize));

numx = uint64(maxbx - minbx + 1);
numy = uint64(maxby - minby + 1);
numxy = numx * numy;

indexVecIdx     = zeros(numPoints, 3, 'uint64');
sortIndexVecIdx = zeros(numPoints, 3, 'uint64');

numClouds = numel(tformedPtCloudArr);

count = tformedPtCloudArr(1).Count;
xyz   = tformedPtCloudArr(1).Location;

ijk0 = uint64(floor(double((xyz(:,1) - xmin)).* inverseVoxelSize));
ijk1 = uint64(floor(double((xyz(:,2) - ymin)).* inverseVoxelSize));
ijk2 = uint64(floor(double((xyz(:,3) - zmin)).* inverseVoxelSize));

voxelIndex = ijk0 + ijk1.*numx + ijk2.*numxy;
strtIdx = 1; endIdx = count;
indexVecIdx(strtIdx:endIdx, 1) = uint64(voxelIndex);
indexVecIdx(strtIdx:endIdx, 2) = uint32(1);
indexVecIdx(strtIdx:endIdx, 3) = uint32(1:count);
if numClouds > 1
    for idx = 2:numClouds
        count = tformedPtCloudArr(idx).Count;
        strtIdx = endIdx + 1;
        endIdx  = endIdx + count;
        xyz = tformedPtCloudArr(idx).Location;
        ijk0 = uint64(floor(double((xyz(:,1) - xmin)).* inverseVoxelSize));
        ijk1 = uint64(floor(double((xyz(:,2) - ymin)).* inverseVoxelSize));
        ijk2 = uint64(floor(double((xyz(:,3) - zmin)).* inverseVoxelSize));
        
        voxelIndex = ijk0 + ijk1.*numx + ijk2.*numxy;
        
        indexVecIdx(strtIdx:endIdx, 1) = uint64(voxelIndex);
        indexVecIdx(strtIdx:endIdx, 2) = uint64(idx);
        indexVecIdx(strtIdx:endIdx, 3) = uint64(1:count);
        
    end
    
end

[sortIdxVec,indices] = sort(indexVecIdx(:,1), 1);
sortIndexVecIdx(:,1) = sortIdxVec;

for k = 1: numPoints
    sortIndexVecIdx(k,2,:) = indexVecIdx(indices(k),2,:) - uint64(1);
    sortIndexVecIdx(k,3,:) = indexVecIdx(indices(k),3,:) - uint64(1);
end

total = numel(unique(sortIndexVecIdx(:,1)));
end
%//////////////////////////////////////////////////////////////////////////
%//  Compute limits form point clouds
%///////////////////////////////////////////////////////////////////////////
function limits = findLimits(limits, tformedClouds, n)

limits(1) = min(limits(1), tformedClouds(n).XLimits(1));
limits(2) = max(limits(2), tformedClouds(n).XLimits(2));
limits(3) = min(limits(3), tformedClouds(n).YLimits(1));
limits(4) = max(limits(4), tformedClouds(n).YLimits(2));
limits(5) = min(limits(5), tformedClouds(n).ZLimits(1));
limits(6) = max(limits(6), tformedClouds(n).ZLimits(2));

end
