function points3D = reconstructFromDisparity(disparityMap, reprojectionMatrix)

% Copyright 2021 MathWorks, Inc.

%#codegen

numPoints = numel(disparityMap);

[x, y] = meshgrid(...
    cast(1:size(disparityMap, 2), 'like', disparityMap),...
    cast(1:size(disparityMap, 1), 'like', disparityMap));
points2dHomog = [x(:), y(:), disparityMap(:), ...
    ones(numPoints, 1, 'like', disparityMap)];
points3dHomog = points2dHomog * reprojectionMatrix;
points3d = bsxfun(@times, points3dHomog(:, 1:3), 1./points3dHomog(:, 4));

% create outputs
X = reshape(points3d(:, 1), size(disparityMap));
Y = reshape(points3d(:, 2), size(disparityMap));
Z = reshape(points3d(:, 3), size(disparityMap));

% invalid disparity results in the 3D location being NaN.
X(disparityMap == -realmax('single')) = NaN;
Y(disparityMap == -realmax('single')) = NaN;
Z(disparityMap == -realmax('single')) = NaN;

points3D = cat(3, X, Y, Z);
end