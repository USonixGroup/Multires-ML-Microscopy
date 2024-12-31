function [filteredLocation, filteredColor, filteredNormal, filteredIntensity, filteredRangeData, varargout] = voxelGridFilter(location, color, normal, intensity, rangeData, voxelSize, rangeIn, varargin)
%   vision.internal.codegen.pc.voxelGridFilter is used in pcdownsample
%   to sample with gridAverage method. This file generates portable code.
%   Shared library code generation version is found in
%   vision.internal.buildable.voxelGridFilterBuildable.
%
%   Inputs:
%   -------
%    location       : point coordinates.
%    color          : point RGB colors.
%    normal         : point normal directions.
%    intensity      : point grayscale intensities.
%    rangeData      : point range, pitch and yaw data.
%    voxelSize      : size of 3-D box for grid filter.
%    rangeIn        : X, Y, Z coordinate limits, recalculate if empty.
%    minVoxelPoints : maximum number of points in the grid box.
%
%   Outputs:
%   -------
%    filteredLocation  : filtered point coordinates.
%    filteredColor     : filtered point RGB colors.
%    filteredNormal    : filtered point normal directions.
%    filteredIntensity : filtered point grayscale intensities.
%    filteredRangeData : filtered point range, pitch and yaw data.
%    covariance        : covariance of neighbors in voxels.
%    voxelCounts       : number of neighbors in voxels.

% Copyright 2021 The MathWorks, Inc.
%#codegen

    coder.inline('never');

    minVoxelPoints = 1;
    if nargin >= 8
        minVoxelPoints = varargin{1};
    end
    
    needXYZLimits     = isempty(rangeIn);
    needCovariance    = nargout >= 6;
    needCount         = nargout >= 7;
    needColorFlag     = ~isempty(color);
    needNormalFlag    = ~isempty(normal);
    needIntensityFlag = ~isempty(intensity);
    needRangeFlag     = ~isempty(rangeData);
    
    coder.internal.prefer_const(minVoxelPoints,needXYZLimits,needCovariance,needCount,needColorFlag,needNormalFlag,needIntensityFlag,needRangeFlag);
    
    if ismatrix(location)
        numPoints = uint32(size(location, 1));
    else
        numPoints = uint32(size(location, 1) * size(location, 2));
    end
    
    inverseVoxelSize = 1 / voxelSize;
        
    if needXYZLimits
        range = coder.nullcopy(zeros([6 1], 'like', location));
        range(1) = min(location(1:numPoints));
        range(2) = max(location(1:numPoints));
        range(3) = min(location(numPoints+1:2*numPoints));
        range(4) = max(location(numPoints+1:2*numPoints));
        range(5) = min(location(2*numPoints+1:3*numPoints));
        range(6) = max(location(2*numPoints+1:3*numPoints));
    else
        range = rangeIn;
    end
    
    % Check that the voxel size is not too small, given the size of the data.
    dx = uint64((range(2) - range(1)) * inverseVoxelSize + 1);
    dy = uint64((range(4) - range(3)) * inverseVoxelSize + 1);
    dz = uint64((range(6) - range(5)) * inverseVoxelSize + 1);
    
    dxy = dx * dy;
    dxyz = dxy * dz;
    
    if (dx == 0 || dy == 0 || dz == 0 || dxy == 0)
        coder.internal.error('vision:pointcloud:voxelSizeTooSmall');
    end
    
    if (dx ~= dxy / dy || dz ~= dxyz / dxy)
        coder.internal.error('vision:pointcloud:voxelSizeTooSmall');
    end
    
    [numOut,ptrIndexVector] = populateIndexVector(location,numPoints,voxelSize,range,needXYZLimits,minVoxelPoints);
    
    filteredLocation = coder.nullcopy(zeros(numOut, 3, 'like', location));
    
    if needColorFlag
        filteredColor = coder.nullcopy(zeros(numOut, 3, 'like', color));
    else
        filteredColor = coder.nullcopy(zeros(0, 3, 'like', color));
    end
    
    if needNormalFlag
        filteredNormal = coder.nullcopy(zeros(numOut, 3, 'like', location));
    else
        filteredNormal = coder.nullcopy(zeros(0, 3, 'like', location));
    end
    
    if needIntensityFlag
        filteredIntensity = coder.nullcopy(zeros(numOut, 1, 'like', intensity));
    else
        filteredIntensity = coder.nullcopy(zeros(0, 1, 'like', intensity));
    end
    
    if needRangeFlag
        filteredRangeData = coder.nullcopy(zeros(numOut, 3, 'like', location));
    else
        filteredRangeData = coder.nullcopy(zeros(0, 3, 'like', location));
    end
    
    if needCovariance
        covariance = coder.nullcopy(zeros(3, 3, numOut,'like', location));
    else
        covariance = coder.nullcopy(zeros(3, 3, 0, 'like', location));
    end
    
    if needCount
        voxelCounts = coder.nullcopy(zeros(numOut, 1, 'uint32'));
    else
        voxelCounts = coder.nullcopy(zeros(0, 1, 'uint32'));
    end

    [filteredLocation,filteredColor,filteredNormal,filteredIntensity,filteredRangeData,covariance,voxelCounts] = voxelGridAlgImpl(...
            location, numPoints, color, normal, intensity, rangeData, range, needXYZLimits, ...
            filteredLocation, filteredColor, filteredNormal, filteredIntensity, filteredRangeData, covariance, voxelCounts, ...
            ptrIndexVector, numOut, ...
            needColorFlag, needNormalFlag, needIntensityFlag, needRangeFlag, needCovariance, needCount);
    
    if needCovariance
        varargout{1} = covariance;
        if needCount
            varargout{2} = voxelCounts;
        end
    end
end

function [numOut,ptrIndexVector] = populateIndexVector(location,numPoints,voxelSize,range,needXYZLimits,minVoxelPoints)
    coder.internal.prefer_const(needXYZLimits,minVoxelPoints);
    
    inverseVoxelSize = 1 / voxelSize;
    xmin = range(1);
    xmax = range(2);
    ymin = range(3);
    ymax = range(4);
    zmin = range(5);
    zmax = range(6);
    
    minbx = coder.internal.indexInt(floor(xmin * inverseVoxelSize));
    maxbx = coder.internal.indexInt(floor(xmax * inverseVoxelSize));
    minby = coder.internal.indexInt(floor(ymin * inverseVoxelSize));
    maxby = coder.internal.indexInt(floor(ymax * inverseVoxelSize));

    numx = double(maxbx - minbx + 1);
    numy = double(maxby - minby + 1);
    numxy = numx * numy;

    ptrIndexVectorTmp = coder.nullcopy(zeros([numPoints 2],'uint64'));
    ptrIndexVectorTmpEnd = coder.internal.indexInt(1);
    numOut = coder.internal.indexInt(0);
    
    if needXYZLimits
        ptrIndexVector = coder.nullcopy(zeros([numPoints 2],'uint64'));
        ptrIndexVectorEnd = coder.internal.indexInt(numPoints);
    
        for n = 1:ptrIndexVectorEnd
            x = location(n);
            y = location(n + ptrIndexVectorEnd);
            z = location(n + ptrIndexVectorEnd * 2);
    
            % Compute the linear voxel index.
            % Adding double casting to match the C++ behavior. In C++
            % inverseVoxelSize is double so computation is in double.
            % In M code or generated code, computation is in x/y/z's
            % type, which could be single.
            ijk0 = floor(double(x - xmin) * inverseVoxelSize);
            ijk1 = floor(double(y - ymin) * inverseVoxelSize);
            ijk2 = floor(double(z - zmin) * inverseVoxelSize);
            voxelIndex = uint64(ijk0 + ijk1 * numx + ijk2 * numxy);
    
            ptrIndexVector(n,:) = [voxelIndex, n];
        end
    else
        ptrIndexVectorCpy = coder.nullcopy(zeros([numPoints 2],'uint64'));
        ptrIndexVectorEnd = coder.internal.indexInt(1);
        % First pass: Go over all points and insert them into the vector
        % with linear voxel index. Points with the same voxel index value
        % will contribute to the same point in the output.
        for n = 1:numPoints
            x = location(n);
            y = location(n + numPoints);
            z = location(n + numPoints * 2);
            
            % A point can only be out of the XYZ limits if the limits were given
            if x < xmin || x > xmax || y < ymin || y > ymax || z < zmin || z > zmax
                numOut = numOut + 1;
                continue;
            end
            
            % Compute the linear voxel index.
            % Adding double casting to match the C++ behavior. In C++
            % inverseVoxelSize is double so computation is in double.
            % In M code or generated code, computation is in x/y/z's
            % type, which could be single.
            ijk0 = floor(double(x - xmin) * inverseVoxelSize);
            ijk1 = floor(double(y - ymin) * inverseVoxelSize);
            ijk2 = floor(double(z - zmin) * inverseVoxelSize);
            voxelIndex = uint64(ijk0 + ijk1 * numx + ijk2 * numxy);

            ptrIndexVectorCpy(ptrIndexVectorEnd,:) = [voxelIndex, n];
            ptrIndexVectorEnd = ptrIndexVectorEnd + 1;
        end
        ptrIndexVectorEnd = ptrIndexVectorEnd-1;
        ptrIndexVector = ptrIndexVectorCpy(1:ptrIndexVectorEnd,:);
    end

    % Second pass, sort the ptrIndexVector vector using voxel index value,
    % so that all points belonging to the same output voxel will be
    % arranged together.
    ptrIndexVector = sortVoxelIndex(ptrIndexVector,coder.internal.indexInt(1),ptrIndexVectorEnd,ptrIndexVectorEnd);
    
    % Third pass, count the number of voxels that have minimum number of points.
    index = 1;
    while index <= ptrIndexVectorEnd
        i = index + 1;
        while (i <= ptrIndexVectorEnd && ptrIndexVector(i,1) == ptrIndexVector(index,1))
            i = i + 1;
        end
        if i-index >= minVoxelPoints
            numOut = numOut + 1;
            if minVoxelPoints > 1
                for j = index:i-1
                    ptrIndexVectorTmp(ptrIndexVectorTmpEnd,:) = ptrIndexVector(j,:);
                    ptrIndexVectorTmpEnd = ptrIndexVectorTmpEnd+1;
                end
            end
        end
        index = i;
    end

    if minVoxelPoints > 1
        ptrIndexVector = ptrIndexVectorTmp(1:ptrIndexVectorTmpEnd-1,:);
    end
end

function [filteredLocation,filteredColor,filteredNormal,filteredIntensity,filteredRangeData,covariance,voxelCounts] = voxelGridAlgImpl(...
                location,nPoints,color,normal,intensity,rangeData,range,needXYZLimits, ...
                filteredLocation,filteredColor,filteredNormal,filteredIntensity,filteredRangeData, ...
                covariance,voxelCounts,ptrIndexVector,nOut, ...
                needColorFlag,needNormalFlag,needIntensityFlag,needRangeFlag,needCovariance,needCount)
    coder.internal.prefer_const(needXYZLimits);
    
    xmin = range(1);
    xmax = range(2);
    ymin = range(3);
    ymax = range(4);
    zmin = range(5);
    zmax = range(6);

    cr = single(0);
    cg = single(0);
    cb = single(0);
    nx = coder.nullcopy(zeros(1, 'like', location));
    ny = coder.nullcopy(zeros(1, 'like', location));
    nz = coder.nullcopy(zeros(1, 'like', location));
    ix = double(0);
    
    % Go through each voxel and calculate average point properties.
    index = coder.internal.indexInt(1);
    n = coder.internal.indexInt(1);
    numPoints = coder.internal.indexInt(nPoints);
    numOut = coder.internal.indexInt(nOut);
    
    while n <= size(ptrIndexVector,1)
        matrixIndex = coder.internal.indexInt(ptrIndexVector(n,2));
        % Initialize location accumulator value to the first point in voxel.
        cx = (location(matrixIndex));
        cy = (location(matrixIndex + numPoints));
        cz = (location(matrixIndex + numPoints * 2));
        if needColorFlag
            % Initialize color accumulator value to the first point in voxel.
            cr = single(color(matrixIndex));
            cg = single(color(matrixIndex + numPoints));
            cb = single(color(matrixIndex + numPoints * 2));
        end
        if needNormalFlag
            % Initialize normal accumulator value to the first point in voxel.
            nx = normal(matrixIndex);
            ny = normal(matrixIndex + numPoints);
            nz = normal(matrixIndex + numPoints * 2);
        end
        if needIntensityFlag
            % Initialize intensity accumulator value to the first point in voxel.
            ix = double(intensity(matrixIndex));
        end
        
        i = n + 1;
        while (i <= size(ptrIndexVector,1) && ptrIndexVector(i,1) == ptrIndexVector(n,1))
            matrixIndex = coder.internal.indexInt(ptrIndexVector(i,2));
            % Use cx, cy, cz to accumulate point coordinates in voxel.
            cx = cx + location(matrixIndex);
            cy = cy + location(matrixIndex + numPoints);
            cz = cz + location(matrixIndex + numPoints * 2);
            if (needColorFlag)
                % Use cr, cg, cb to accumulate point colors in voxel.
                cr = cr + single(color(matrixIndex));
                cg = cg + single(color(matrixIndex + numPoints));
                cb = cb + single(color(matrixIndex + numPoints * 2));
            end
            if (needNormalFlag)
                % Use nx, ny, nz to accumulate point normals in voxel.
                nx = nx + normal(matrixIndex);
                ny = ny + normal(matrixIndex + numPoints);
                nz = nz + normal(matrixIndex + numPoints * 2);
            end
            if (needIntensityFlag)
                % Use ix to accumulate point intensities in voxel.
                ix = ix + double(intensity(matrixIndex));
            end
            i = i + 1;
        end
        
        % Calculate average point coordinates in voxel and store in output.
        cx = cx / double(i - n);
        cy = cy / double(i - n);
        cz = cz / double(i - n);
        filteredLocation(index) = cx;
        filteredLocation(index + numOut) = cy;
        filteredLocation(index + numOut * 2) = cz;

        if (needColorFlag)
            % Calculate average color in voxel and store in output.
            cr = cr / double(i - n);
            cg = cg / double(i - n);
            cb = cb / double(i - n);
            filteredColor(index) = cast(floor(cr),'like',filteredColor(index));
            filteredColor(index + numOut) = cast(floor(cg),'like',filteredColor(index));
            filteredColor(index + numOut * 2) = cast(floor(cb),'like',filteredColor(index));
        end
        if (needNormalFlag)
            % Calculate average normal in voxel and store in output.
            nx = nx / double(i - n);
            ny = ny / double(i - n);
            nz = nz / double(i - n);
            filteredNormal(index) = nx;
            filteredNormal(index + numOut) = ny;
            filteredNormal(index + numOut * 2) = nz;
        end
        if (needIntensityFlag)
            % Calculate average intensity in voxel and store in output.
            ix = ix / double(i - n);
            if isfloat(filteredIntensity)
                filteredIntensity(index) = cast(ix,'like',filteredIntensity(index));
            else
                filteredIntensity(index) = cast(floor(ix),'like',filteredIntensity(index));
            end
        end
        if (needRangeFlag)
            % Calculate average range in voxel and store in output.
            range = sqrt(cx * cx + cy * cy + cz * cz);
            pitch = asin(cz / range);
            yaw = atan2(cx, cy);
            if (yaw < 0)
                yaw = yaw + 2 * pi;
            end
            filteredRangeData(index) = range;
            filteredRangeData(index + numOut) = pitch;
            filteredRangeData(index + numOut * 2) = yaw;
        end

        n = i;
        index = index + 1;
    end
    
    % Compute covariance, if needed.
    if (needCovariance || needCount)
        index = coder.internal.indexInt(1);
        n = 1;
        while n <= size(ptrIndexVector,1)
            % Compute number of neighbors in the voxel.
            i = n + 1;
            while (i <= size(ptrIndexVector,1) && ptrIndexVector(i,1) == ptrIndexVector(n,1))
                i = i + 1;
            end
            numNeighbors = i - n;
            
            % Compute covariance for the voxel and store in output.
            covariance = iCovarianceMatrix(filteredLocation, location, n, numPoints, numOut, numNeighbors, index, ptrIndexVector, covariance);
            
            if (needCount)
                voxelCounts(index) = numNeighbors;
            end

            n = i;
            index = index + 1;
        end
    end
    
    % Don't need to copy untouched data if the XYZ limits are calculated, since
    % data can only be outside the range if the range is given as input
    if needXYZLimits
        return;
    end
    
    % Copy the untouched data outside the given range.
    for n = 1:numPoints
        x = location(n);
        y = location(n + numPoints);
        z = location(n + numPoints * 2);

        if (x < xmin || x > xmax || y < ymin || y > ymax || z < zmin || z > zmax)
            % Copy coordinates of untouched points.
            filteredLocation(index) = x;
            filteredLocation(index + numOut) = y;
            filteredLocation(index + numOut * 2) = z;

            if (needColorFlag)
                % Copy colors of untouched points.
                filteredColor(index) = color(n);
                filteredColor(index + numOut) = color(n + numPoints);
                filteredColor(index + numOut * 2) = color(n + numPoints * 2);
            end
            if (needNormalFlag)
                % Copy normals of untouched points.
                filteredNormal(index) = normal(n);
                filteredNormal(index + numOut) = normal(n + numPoints);
                filteredNormal(index + numOut * 2) = normal(n + numPoints * 2);
            end
            if (needIntensityFlag)
                % Copy intensities of untouched points.
                filteredIntensity(index) = intensity(n);
            end
            if (needRangeFlag)
                % Copy ranges of untouched points.
                filteredRangeData(index) = rangeData(n);
                filteredRangeData(index + numOut) = rangeData(n + numPoints);
                filteredRangeData(index + numOut * 2) = rangeData(n + numPoints * 2);
            end
            if (needCovariance)
                % Copy covariances of untouched points.
                for d = 1:9
                    covariance((index - 1) * 9 + coder.internal.indexInt(d)) = nan;
                end
            end
            index = index + 1;
        end
    end
end

function covariance = iCovarianceMatrix(filteredLocation, location, n, numPoints, numOut, numNeighbors, index, ptrIndexVector, covariance)
    % Compute covariance in the voxel and store in the output.
    coder.inline("always");
    
    cov = zeros(3);
    % Get centroid of current voxel.
    cx = filteredLocation(index);
    cy = filteredLocation(index + numOut);
    cz = filteredLocation(index + numOut * 2);
    
    if (numNeighbors > 1)
        for k = 0:numNeighbors-1
            matrixIndex = ptrIndexVector(n+k,2);
            xVal = location(matrixIndex);
            yVal = location(matrixIndex + uint64(numPoints));
            zVal = location(matrixIndex + uint64(numPoints)*2);
            cov(1) = cov(1) + xVal * xVal;
            cov(2) = cov(2) + xVal * yVal;
            cov(3) = cov(3) + xVal * zVal;
            cov(4) = cov(4) + xVal * yVal;
            cov(5) = cov(5) + yVal * yVal;
            cov(6) = cov(6) + yVal * zVal;
            cov(7) = cov(7) + zVal * xVal;
            cov(8) = cov(8) + zVal * yVal;
            cov(9) = cov(9) + zVal * zVal;
        end

        sx = cx * numNeighbors;
        sy = cy * numNeighbors;
        sz = cz * numNeighbors;

        cov(1) = cov(1) - 2 * sx * cx + numNeighbors * cx * cx;
        cov(2) = cov(2) - 2 * sx * cy + numNeighbors * cx * cy;
        cov(3) = cov(3) - 2 * sx * cz + numNeighbors * cx * cz;
        cov(4) = cov(4) - 2 * sy * cx + numNeighbors * cx * cy;
        cov(5) = cov(5) - 2 * sy * cy + numNeighbors * cy * cy;
        cov(6) = cov(6) - 2 * sy * cz + numNeighbors * cy * cz;
        cov(7) = cov(7) - 2 * sz * cx + numNeighbors * cz * cx;
        cov(8) = cov(8) - 2 * sz * cy + numNeighbors * cz * cy;
        cov(9) = cov(9) - 2 * sz * cz + numNeighbors * cz * cz;
        
        cov = cov / (numNeighbors - 1);
    else
        cov = cov * nan;
    end

    covariance((index - 1) * 9 + 1) = cov(1);
    covariance((index - 1) * 9 + 2) = cov(4);
    covariance((index - 1) * 9 + 3) = cov(7);
    covariance((index - 1) * 9 + 4) = cov(2);
    covariance((index - 1) * 9 + 5) = cov(5);
    covariance((index - 1) * 9 + 6) = cov(8);
    covariance((index - 1) * 9 + 7) = cov(3);
    covariance((index - 1) * 9 + 8) = cov(6);
    covariance((index - 1) * 9 + 9) = cov(9);
end

function x = sortVoxelIndex(x,xstart,xend,arrLen)
% This function uses a combination of quick sort and insertion sort to
% sort an Nx2 matrix from row xstart to xend. Sorting is based on the
% value of the first column.
%#codegen
    coder.inline('never');
    if xstart >= xend
        % Done
        return
    end
    % Insertion sort is faster for small intervals.
    BND = 32;
    if xend - xstart + 1 <= BND
        x = insertion(x,xstart,xend,arrLen);
        return
    end
    frame = struct('xstart',xstart,'xend',xend);
    st = coder.internal.stack(frame);
    st = st.push(frame);
    while st.stackSize() > 0
        [s,st] = st.pop();
        xstart = s.xstart; xend = s.xend;
        if xend - xstart + 1 <= BND
            x = insertion(x,xstart,xend,arrLen);
        else
            [p,x] = sortpartition(x,xstart,xend,arrLen);
            % Push right then left partition to match the customary implementation's
            % recursion order
            if p+1 < xend
                st = st.push(struct('xstart',p+1,'xend',xend));
            end
            if xstart < p
                st = st.push(struct('xstart',xstart,'xend',p));
            end
        end
    end
end

function x = insertion(x,xstart,xend,arrLen)
% This function uses insertion sort to sort an Nx2 matrix from row xstart 
% to xend. Sorting is based on the value of the first column.
%#codegen
    coder.inline('never');
    for k = xstart+1:xend
        xc = x(k);
        xc2 = x(k+arrLen);
        idx = k-1;
        while idx >= xstart && xc < x(idx)
            x(idx+1) = x(idx);
            x(idx+arrLen+1) = x(idx+arrLen);
            idx = idx-1;
        end
        x(idx+1) = xc;
        x(idx+arrLen+1) = xc2;
    end
end

function [p,x] = sortpartition(x,xstart,xend,arrLen)
% This function performs the partition step in quicksort: partitions the 
% input Nx2 matrix x using the median-of-3 as the pivot. 
%#codegen
    xmid = xstart + coder.internal.indexDivide(xend-xstart,2);
    
    % Find median of first, middle, last, and use as pivot.
    diff1 = double(x(xstart)) - double(x(xmid));
    diff2 = double(x(xmid))   - double(x(xend));
    diff3 = double(x(xstart)) - double(x(xend));
    if (diff1 * diff2 > 0)
        pivot = x(xmid);
        pivot2 = x(xmid+arrLen);
        pivotIdx = xmid;
    elseif (diff1 * diff3 > 0)
        pivot = x(xend);
        pivot2 = x(xend+arrLen);
        pivotIdx = xend;
    else
        pivot = x(xstart);
        pivot2 = x(xstart+arrLen);
        pivotIdx = xstart;
    end
    % Swap the pivot to the end.
    x(pivotIdx) = x(xend);
    x(pivotIdx+arrLen) = x(xend+arrLen);
    x(xend) = pivot;
    x(xend+arrLen) = pivot2;

    % partition x(xstart:xend-1) such that pivot is at the correct position.
    i = xstart-1;
    j = xend;
    while true
        i = i+1;
        while x(i) < pivot
            i = i+1;
        end
        j = j-1;
        while pivot < x(j)
            j = j-1;
        end
        if i >= j
            p = i;
            x(xend) = x(i);
            x(xend+arrLen) = x(i+arrLen);
            x(i) = pivot;
            x(i+arrLen) = pivot2;
            return
        else
            t = x(i);
            t2 = x(i+arrLen);
            x(i) = x(j);
            x(i+arrLen) = x(j+arrLen);
            x(j) = t;
            x(j+arrLen) = t2;
        end
    end
end
