function ptCloudOut = pccat(pc)
%   vision.internal.codegen.pc.pccat Concatenate 3-D point clouds for code
%   generation
%   ptCloudOut = PCCAT(ptClouds) concatenates the array of point clouds
%   in ptClouds into a single point cloud ptCloudOut containing the
%   Location, Color, Normal and Intensity properties of all point clouds.

% Copyright 2020-2022 The MathWorks, Inc.
%#codegen

numClouds      = numel(pc);
hasColor       = true;
hasIntensity   = true;
hasNormal      = true;

% Getting the dataTypes of location and intensity
locationDataType  = class(pc(1).Location);
intensityDataType = class(pc(1).Intensity);
colorDataType     = class(pc(1).Color);


% Arrays to hold the pointCloud data
locationArray  = cast([], locationDataType);
colorArray     = cast([], colorDataType);
normalArray    = cast([], locationDataType);
intensityArray = cast([], intensityDataType);

for i = 1:numClouds
    [points, color, normal, intensity] = extractValidPoints(pc(i));
    
    locationArray  = [locationArray;points];     %#ok
    colorArray     = [colorArray;color];         %#ok
    normalArray    = [normalArray;normal];       %#ok
    intensityArray = [intensityArray;intensity]; %#ok
    
    if size(points,1) ~= 0
        hasColor          = hasColor && ~isempty(color);
        hasIntensity      = hasIntensity && ~isempty(intensity);
        hasNormal         = hasNormal && ~isempty(normal);
    end
end

if hasColor && numel(colorArray) == numel(locationArray)
    color = colorArray;
else
    color = cast(zeros(0,0), colorDataType);
end

if hasNormal && numel(normalArray) == numel(locationArray)
    normal = normalArray;
else
    normal  = cast(zeros(0,0), locationDataType);
end

if hasIntensity && size(intensityArray,1) == size(locationArray,1)
    intensity = intensityArray;
else
    intensity = cast(zeros(0,0), intensityDataType);
end

% Make point cloud
ptCloudOut = pointCloud(locationArray, 'Color', color, 'Normal', normal,...
    'Intensity', intensity);

end
