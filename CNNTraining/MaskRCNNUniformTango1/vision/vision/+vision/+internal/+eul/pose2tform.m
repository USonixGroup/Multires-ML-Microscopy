%vision.internal.eul.pose2tform Convert pose to rigidtform3d transformation
%   tform = pose2tform(xyzeul) converts a 6-element pose vector [x, y, z,
%   phix, phiy, phiz] to a rigidtform3d object.
%
%   tform = pose2tform(xyzeul, type) additionally uses type as the storage
%   format. type can be 'single' or 'double'.
%
%   See also tform2pose.

% Copyright 2019-2022 The MathWorks, Inc.

%#codegen
function tform = pose2tform(pose, type)

if ~isGPUCodegen
    trans = cast(pose(1:3), type);
    rot   = cast(vision.internal.eul.eulerAngleToRotationMatrix(pose(4:6)), type);
else
    [rot,trans] = vision.internal.codegen.gpu.pcregisterndt.eulerAngleToRotationMatrix(pose, type);
end

tform = rigidtform3d(rot, trans);
end

% GPU Codegen flag
function flag = isGPUCodegen()
flag = coder.gpu.internal.isGpuEnabled;
end
