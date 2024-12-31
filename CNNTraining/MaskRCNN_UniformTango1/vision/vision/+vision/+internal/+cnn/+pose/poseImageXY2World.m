function centroids3DXY = poseImageXY2World(centroids2DXY,Zcentroids,K)
%POSEIMAGEXY2WORLD Project 2D image points to 3D given the Zs and intrinsics matrix K

% Copyright 2023 The MathWorks, Inc.

    uvz = cat(2, centroids2DXY .* Zcentroids, Zcentroids);
    XYZ = (pinv(K) * uvz')' ;
    centroids3DXY = XYZ(:, 1:2);
end