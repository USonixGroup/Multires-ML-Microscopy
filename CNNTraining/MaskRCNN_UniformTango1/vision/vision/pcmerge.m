function ptCloudOut = pcmerge(ptCloudA, ptCloudB, gridStep)
%

% Copyright 2014-2024 The MathWorks, Inc.

%#codegen

% Validate input arguments
validateattributes(ptCloudA, {'pointCloud'}, {'scalar'}, mfilename, 'ptCloudA');
validateattributes(ptCloudB, {'pointCloud'}, {'scalar'}, mfilename, 'ptCloudB');
validateattributes(gridStep, {'single', 'double'},{'real', 'nonsparse', 'scalar', 'nonnan', 'positive'}, mfilename, 'gridStep');

if ~isGPUTarget()
    % Simulation and C Codegen implementation path
    % Remove invalid points to determine a bounded volume
    [pointsA, colorA, normalA, intensityA, rangeDataA] = ptCloudA.extractValidPoints();
    [pointsB, colorB, normalB, intensityB, rangeDataB] = ptCloudB.extractValidPoints();

    if isempty(pointsA) && isempty(pointsB)
        ptCloudOut = createPointCloud(pointsA, colorA, normalA, intensityA, rangeDataA);
    elseif isempty(pointsA) && ~isempty(pointsB)
        ptCloudOut = createPointCloud(pointsB, colorB, normalB, intensityB, rangeDataB);
    elseif isempty(pointsB) && ~isempty(pointsA)
        ptCloudOut = createPointCloud(pointsA, colorA, normalA, intensityA, rangeDataA);
    else

        points = vertcat(pointsA, pointsB);
        color = vertcat(colorA, colorB);
        if numel(color) ~= numel(points)
            color = cast([], 'like', color);
        end
        normal = vertcat(normalA, normalB);
        if numel(normal) ~= numel(points)
            normal = cast([], 'like', points);
        end
        intensity = vertcat(intensityA, intensityB);
        if ~(size(intensity, 1) == size(points, 1) && size(intensity, 2) == 1)
            intensity = cast([], 'like', points);
        end
        rangeData = vertcat(rangeDataA, rangeDataB);
        if numel(rangeData) ~= numel(points)
            rangeData = cast([], 'like', points);
        end

        overlapLimits = overlapRange(pointsA, pointsB);

        if isempty(overlapLimits)
            ptCloudOut = createPointCloud(points, color, normal, intensity, rangeData);
        else
            if isempty(coder.target)
                % Apply grid filter to each property
                [points, color, normal, intensity, rangeData] = ...
                    visionVoxelGridFilter(points, color, normal, intensity, ...
                    rangeData, gridStep, overlapLimits);
            elseif coder.internal.preferMATLABHostCompiledLibraries()
                [points, color, normal, intensity, rangeData] = ...
                    vision.internal.buildable.voxelGridFilterBuildable.voxelGridFilter(...
                    points, color, normal, intensity, rangeData, gridStep, ...
                    overlapLimits);
            else
                [points, color, normal, intensity, rangeData] = vision.internal.codegen.pc.voxelGridFilter(...
                    points, color, normal, intensity, rangeData, gridStep, ...
                    overlapLimits);
            end
            ptCloudOut = createPointCloud(points, color, normal, intensity, rangeData);
        end
    end
else
    % GPU Codegen implementation of pcmerge
    [outLoc,outCol,outNorm,outInt,outRData] = vision.internal.codegen.gpu.pcmergeGPUImpl(...
        ptCloudA.Location,ptCloudA.Color,ptCloudA.Intensity,...
        ptCloudA.Normal,ptCloudA.RangeData,...
        ptCloudB.Location,ptCloudB.Color,ptCloudB.Intensity,...
        ptCloudB.Normal,ptCloudB.RangeData, gridStep);

    ptCloudOut = pointCloud(outLoc, 'Color', outCol, ...
        'Normal', outNorm, 'Intensity', outInt);
    ptCloudOut.RangeData = outRData;
end

end

%--------------------------------------------------------------------------
function ptCloud = createPointCloud(points, color, normals, intensity, rangeData)

ptCloud = pointCloud(points, 'Color', color, 'Normal', normals, ...
    'Intensity', intensity);
ptCloud.RangeData = rangeData;
end

%--------------------------------------------------------------------------
function rangeLimits = overlapRange(pointsA, pointsB)

xlimA = [min(pointsA(:,1)), max(pointsA(:,1))];
ylimA = [min(pointsA(:,2)), max(pointsA(:,2))];
zlimA = [min(pointsA(:,3)), max(pointsA(:,3))];

xlimB = [min(pointsB(:,1)), max(pointsB(:,1))];
ylimB = [min(pointsB(:,2)), max(pointsB(:,2))];
zlimB = [min(pointsB(:,3)), max(pointsB(:,3))];

if (xlimA(1) > xlimB(2) || xlimA(2) < xlimB(1) || ...
        ylimA(1) > ylimB(2) || ylimA(2) < ylimB(1) || ...
        zlimA(1) > zlimB(2) || zlimA(2) < zlimB(1))

    rangeLimits = cast([], 'like', xlimA);
else
    rangeLimits = [ max(xlimA(1),xlimB(1)), min(xlimA(2),xlimB(2)) ...
        max(ylimA(1),ylimB(1)), min(ylimA(2),ylimB(2)) ...
        max(zlimA(1),zlimB(1)), min(zlimA(2),zlimB(2))];
end
end

%--------------------------------------------------------------------------
function flag = isGPUTarget()
    flag = coder.gpu.internal.isGpuEnabled;
end
