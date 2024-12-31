function [unorgPtCloud,indices]= removeInvalidPoints(location)
% vision.internal.codegen.pc.removeInvalidPoints removes invalid points from
% 3-D point clouds for code generation
% This function is used if only the valid locations of the input
% point cloud are required in the output.

% Copyright 2023 The MathWorks, Inc.

%#codegen

newLocations = reshape(location,[],3);
indices = false(size(newLocations,1),1);

% Find the indices of the valid locations in the point cloud
parfor i = 1:size(newLocations,1)
    if isfinite(sum(newLocations(i,:)))
        indices(i) = true;
    end
end

% Extract valid locations from the original point cloud locations
validLocations = newLocations(indices,:);

unorgPtCloud = pointCloud(validLocations);
end