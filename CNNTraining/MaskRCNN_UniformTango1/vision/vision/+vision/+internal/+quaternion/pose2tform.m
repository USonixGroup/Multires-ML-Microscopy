%vision.internal.quaternion.pose2tform Convert pose to rigid3d transformation
%   tforms = pose2tform(xyzquats) converts an M-by-7 matrix of
%   [x, y, z, q1, q2, q3, q4] poses to an M-element array of rigid3d
%   transformations.
%
%   See also tform2pose.

% Copyright 2019 The MathWorks, Inc.
function tforms = pose2tform(xyzquats)

numPoses = size(xyzquats, 1);

tforms = rigid3d.empty;

trans = permute(xyzquats(:, 1:3), [3 2 1]);
rots  = permute(...
    vision.internal.quaternion.quaternionToRotation(xyzquats(:, 4:end).'), ...
    [2 1 3]); % 3-by-3-by-M

dtype = class(trans);
tmats = [...
    rots, zeros(3, 1, numPoses, dtype); ...
    trans, ones(1, 1, numPoses, dtype)];

for n = 1 : numPoses
    tforms(n) = rigid3d(tmats(:, :, n));
end

%{
trans = xyzquats(:, 1:3);
rots  = permute(...
    vision.internal.quaternion.quaternionToRotation(xyzquats(:, 4:end).'), ...
    [2 1 3]); % 3-by-3-by-M

for n = 1 : numPoses
    tforms(n) = rigid3d(rots(:, :, n), trans(n, :));
end
%}

%{
tforms = repelem(rigid3d, numPoses, 1);

trans = xyzquats(:, 1:3); % M-by-3
rots  = permute(...
    vision.internal.quaternion.quaternionToRotation(xyzquats(:, 4:end).'), ...
    [2 1 3]); % 3-by-3-by-M

for n = 1 : numPoses
    tforms(n).Translation = trans(n, :);
    tforms(n).Rotation    = rots(:, :, n);
end

%}
end