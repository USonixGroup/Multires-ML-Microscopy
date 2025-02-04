function families = supportedAprilTagFamilies()
% supportedAprilTagFamilies Get a list of supported AprilTag families.
%   families = suppotedAprilTagFamilies() returns a cellstr containing the
%   list of all supported AprilTag families in families.

%   Copyright 2020-2021 The MathWorks, Inc.
%#codegen

families = {'tag16h5','tag25h9','tag36h11','tagCircle21h7','tagCircle49h12', ...
    'tagCustom48h12','tagStandard41h12','tagStandard52h13'};

end