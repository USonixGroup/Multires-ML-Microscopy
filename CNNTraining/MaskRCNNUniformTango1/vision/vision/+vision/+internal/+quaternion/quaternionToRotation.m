function R = quaternionToRotation(quats)
% quaternionToRotation Convert unit quaternions to orthogonal rotation matrices
% 
% quaternion is a 4-by-M vector
% R is a 3-by-3-by-M matrix of corresponding orthogonal rotation matrices
%
% Note
% ----
% R is rotation of vectors anti-clockwise in a right-handed system by
% pre-multiplication

% Copyright 2014-2020 The MathWorks, Inc.

% References
% ----------
% http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation#From_a_quaternion_to_an_orthogonal_matrix

%#codegen

numQuats = size(quats, 2);

q0 = reshape(quats(1, :), 1, 1, numQuats);
qx = reshape(quats(2, :), 1, 1, numQuats);
qy = reshape(quats(3, :), 1, 1, numQuats);
qz = reshape(quats(4, :), 1, 1, numQuats);

R = [q0.^2+qx.^2-qy.^2-qz.^2, 2*qx.*qy-2*q0.*qz,       2*qx.*qz+2*q0.*qy; ...
     2*qx.*qy+2*q0.*qz,       q0.^2-qx.^2+qy.^2-qz.^2, 2*qy.*qz-2*q0.*qx; ...
     2*qx.*qz-2*q0.*qy,       2*qy.*qz+2*q0.*qx,       q0.^2-qx.^2-qy.^2+qz.^2];
