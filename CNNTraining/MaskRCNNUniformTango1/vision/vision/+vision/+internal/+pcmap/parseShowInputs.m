%vision.internal.pcmap.parseShowInputs Parse inputs for the
%show method of the map representations.

% Copyright 2020 The MathWorks, Inc.

function [spatialExtent, extraArgs] = parseShowInputs(spatialExtent, args)

arguments
    spatialExtent(1,1) string {mustBeMember(spatialExtent, ["submap", "map"])} = "map";    
    args.MarkerSize(1,1) = 6;
    args.Parent = [];
end

extraArgs = {'MarkerSize', args.MarkerSize};
if ~isempty(args.Parent)
    extraArgs{end+1} = 'Parent';
    extraArgs{end+1} = args.Parent;
end

end