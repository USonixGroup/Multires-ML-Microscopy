function [indices, dists, valid] = multiQueryRadiusSearchImpl(locations, querypoints, radius)
% codegen support for multi query radius search.

% Copyright 2019 The MathWorks, Inc.

%#codegen

% Validate the inputs
validateattributes(locations, {'single', 'double'}, {'real', 'nonsparse' });
coder.internal.errorIf(~((numel(size(locations))==2 && size(locations,2)==3) ...
    || (numel(size(locations))==3 && size(locations,3)==3)),'vision:pointcloud:invalidXYZPoints');

validateattributes(querypoints, {'single', 'double'}, ...
    {'real', 'nonsparse', 'nonnan', 'finite', 'size', [NaN, 3]});

validateattributes(radius, {'single', 'double'}, ...
    {'real', 'nonsparse', 'scalar', 'nonnan', 'finite', 'nonnegative'});

ptCloud = pointCloud(locations);
querypoints  = cast(querypoints, class(locations));
radius = double(radius);
allIndices = uint32(zeros(0, 1));
allDists   = zeros(0, 1);
coder.varsize('allIndices');
coder.varsize('allDists');
valid = uint32(zeros(ptCloud.Count, 1));

for loc = 1:ptCloud.Count
    [indloc, distloc] = findNeighborsInRadius(ptCloud, querypoints(loc,:), radius);
    valid(loc)        = length(indloc);
    [distloc, sorLoc] = sort(distloc);
    indloc            = indloc(sorLoc);
    allIndices        = [allIndices; indloc]; %#ok<*AGROW>
    allDists          = [allDists; distloc.^2]; %#ok<*AGROW>
end

validLen = max(valid);
indices  = uint32(zeros(validLen, ptCloud.Count));
dists    = zeros(validLen, ptCloud.Count);
ids      = uint32(1);
ide      = uint32(0);

for loc = 1:ptCloud.Count
    ide = ide + valid(loc);

    indices(1:valid(loc), loc) = allIndices(ids:ide, 1);
    dists(1:valid(loc), loc)   = allDists(ids:ide, 1);
    ids = ide + 1;
end