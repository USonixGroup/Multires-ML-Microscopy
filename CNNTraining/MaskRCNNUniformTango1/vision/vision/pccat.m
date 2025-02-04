function ptCloud = pccat(ptClouds)

% Copyright 2020-2023 The MathWorks, Inc.

%#codegen
validateattributes(ptClouds, {'pointCloud'}, {'vector', 'nonempty'}, mfilename, 'ptClouds');

if isempty(coder.target)
    % Remove invalid points and make unorganized
    [points, colors, normals, intensity, rangeData] = ...
        arrayfun(@extractValidPoints, ptClouds, 'UniformOutput', false);
    
    % Concatenate attributes
    points = vertcat(points{:});
    
    colors = vertcat(colors{:});
    if numel(colors) ~= numel(points)
        colors = cast([], 'uint8');
    end
    
    normals   = vertcat(normals{:});
    if numel(normals) ~= numel(points)
        normals = cast([], 'like', points);
    end
    
    intensity = vertcat(intensity{:});
    if size(intensity,1) ~= size(points,1)
        intensity = cast([], 'like', points);
    end
    
    rangeData = vertcat(rangeData{:});
    if numel(rangeData) ~= numel(points)
        rangeData = cast([], 'like', points);
    end
    
    % Make point cloud
    ptCloud = pointCloud(points, 'Color', colors, 'Normal', normals, 'Intensity', intensity);
    ptCloud.RangeData = rangeData;
else
    ptCloud = vision.internal.codegen.pc.pccat(ptClouds);
end
end
