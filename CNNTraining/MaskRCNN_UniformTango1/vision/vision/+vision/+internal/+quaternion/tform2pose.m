%vision.internal.quaternion.tform2pose Convert rigid3d transformations to pose
%   xyzquats = tform2pose(tforms) converts an array of rigid3d objects to a
%   7-element pose vector [x, y, z, q1, q2, q3, q4].
%
%   See also pose2tform.

% Copyright 2019-2021 The MathWorks, Inc.

%#codegen

function poses = tform2pose(tforms)

% Extract transformation matrix and convert to post-multiply format.
tmats = permute(cat(3, tforms(:).T), [2 1 3]);

xyz  = squeeze(tmats(1:3, end, :)).';
quat = vision.internal.quaternion.rotationToQuaternion(tmats(1:3, 1:3, :));

poses = [xyz, quat.'];
end