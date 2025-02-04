function rangeData = convertFromCartesianToSphericalCoordinate(xyzData)
% convertFromCartesianToSphericalCoordinate Convert from Cartesian to
% spherical coordinate system. The input is a MxNx3 organized point cloud.
% The output is range, pitch, and yaw.

% Copyright 2018-2020 The MathWorks, Inc.
%#codegen

% Check if GPU is not enabled.
if ~isGPUTarget()
    
    X = xyzData(:,:,1);
    Y = xyzData(:,:,2);
    Z = xyzData(:,:,3); 
    range = sqrt(X.^2 + Y.^2 + Z.^2);
    pitch = asin(Z./range);
    yaw = atan2(X, Y);
    yaw(yaw < 0) = yaw(yaw < 0) + 2*pi; % convert to [0, 2*pi]
    rangeData = cat(3, range, pitch, yaw);

% If GPU is enabled.
else 
    % Calling GPU specific implementation of
    % convertFromCartesianToSphericalCoordinate.
    rangeData = vision.internal.codegen.gpu.convertFromCartesianToSphericalCoordinateImpl...
        (xyzData);
end
end

% GPU codegen support flag
function flag = isGPUTarget()
    flag = coder.gpu.internal.isGpuEnabled;
end
