function ptCloud = pcalign(ptClouds, tforms, gridStep)

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen

if nargin>2
    validateGridStep(gridStep);
    gridStep = double(gridStep);
else
    gridStep = [];
end
if isempty(coder.target)
    validIdx = validatePointCloudArray(ptClouds);
    validateTformArray(tforms, size(ptClouds));
    % Invoke builtin
    if ~isempty(ptClouds(validIdx))
        [xyzPoints, colors, normals, intensity, rangeData] = vision.internal.alignPointClouds(...
            ptClouds(validIdx), cat(3, tforms(validIdx).T), gridStep);
    else
        xyzPoints = zeros(0, 3, 'like', ptClouds(1).Location);
        colors    = cast([], 'uint8');
        normals   = cast([], 'like', ptClouds(1).Location);
        intensity = cast([], 'like', ptClouds(1).Location);
        rangeData = cast([], 'like', ptClouds(1).Location);
    end

    % Create point cloud object
    ptCloud = pointCloud(xyzPoints, 'Color', colors, 'Normal', normals, ...
        'Intensity', intensity);
    ptCloud.RangeData = rangeData;
else
    ptCloud = vision.internal.codegen.pc.pcalign(ptClouds, tforms, gridStep);
end
end


%--------------------------------------------------------------------------
function validIdx = validatePointCloudArray(ptClouds)

validateattributes(ptClouds, {'pointCloud'}, {'vector', 'nonempty'}, ...
    'pcalign', 'ptClouds');

% Ignore empty point clouds
validIdx = arrayfun(@(x)~isempty(x.Location), ptClouds);

% Among non-empty point clouds, all must have the same Location type
isSingleIdx = arrayfun(@(x)isa(x.Location, 'single'), ptClouds(validIdx));
isDoubleIdx = arrayfun(@(x)isa(x.Location, 'double'), ptClouds(validIdx));
if ~(all(isSingleIdx) || all(isDoubleIdx))
    error(message('vision:pointcloud:differentTypes'))
end

% Among non-empty point clouds, all must have the same Color type
isUint8Idx = arrayfun(@(x)isa(x.Color, 'uint8'), ptClouds(validIdx));
isUint16Idx = arrayfun(@(x)isa(x.Color, 'uint16'), ptClouds(validIdx));
if ~(all(isUint8Idx) || all(isUint16Idx))
    error(message('vision:pointcloud:differentTypes'))
end

% Among non-empty point clouds, all must have the same type of intensity
% when present
hasIntensity = arrayfun(@(x)~isempty(x.Intensity), ptClouds(validIdx));
isSingleIdx = arrayfun(@(x)isa(x.Intensity, 'single'), ptClouds(validIdx));
isDoubleIdx = arrayfun(@(x)isa(x.Intensity, 'double'), ptClouds(validIdx));
isUint8Idx  = arrayfun(@(x)isa(x.Intensity, 'uint8'), ptClouds(validIdx));
isUint16Idx  = arrayfun(@(x)isa(x.Intensity, 'uint16'), ptClouds(validIdx));
if ~(all(isSingleIdx(hasIntensity)) || all(isDoubleIdx(hasIntensity)) || ...
     all(isUint8Idx(hasIntensity))  || all(isUint16Idx(hasIntensity)))
    error(message('vision:pointcloud:differentTypes'))
end
end

%--------------------------------------------------------------------------
function validateTformArray(tforms, ptCloudsSize)

validateattributes(tforms, {'rigidtform3d','affinetform3d','rigid3d','affine3d'}, ...
    {'vector','size', ptCloudsSize}, 'pcalign', 'tforms');
end

%--------------------------------------------------------------------------
function validateGridStep(gridStep)

validateattributes(gridStep, {'single', 'double'}, ...
    {'scalar', 'real', 'positive', 'nonsparse'}, 'pcalign', 'gridStep');
end
