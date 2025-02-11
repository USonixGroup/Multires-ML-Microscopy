%vision.internal.eul.tform2pose Convert rigidtform3d or rigid3d transformation to pose
%   xyzeul = tform2pose(tform) converts a rigidtform3d object, rigid3d object,
%   or pre-multiply homogeneous transformation matrix to a 6-element pose
%   vector [x, y, z, phix, phiy, phiz].
%
%   See also pose2tform.

% Copyright 2019-2022 The MathWorks, Inc.

%#codegen
function pose = tform2pose(tform)

if isa(tform, 'rigidtform3d') || isa(tform, 'rigid3d')
    rot   = tform.T(1:3, 1:3);
    trans = tform.T(4, 1:3);
    eul = vision.internal.eul.rotationMatrixToEulerAngle(rot'); % Must transpose "post-multiply' rotation matrix
else
    rot   = tform(1:3, 1:3);
    trans = tform(1:3, 4)';     % Transpose to row vector for pose concatenation
    eul = vision.internal.eul.rotationMatrixToEulerAngle(rot);
end

pose = [trans eul];
end