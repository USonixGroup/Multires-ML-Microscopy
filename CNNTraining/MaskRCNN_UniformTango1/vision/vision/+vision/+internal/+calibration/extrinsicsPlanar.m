% extrinsicsPlanar Compute rotation and translation from homography

% Copyright 2017-2022 MathWorks, Inc.

%#codegen
%#ok<*EMCLS>
%#ok<*EMCA>

function [R,T] = extrinsicsPlanar(imagePoints, worldPoints, intrinsics)

A = intrinsics;

% Compute homography.
H = fitgeotrans(worldPoints, imagePoints, 'projective');
H = H.T';
h1 = H(:, 1);
h2 = H(:, 2);
h3 = H(:, 3);

lambda = 1 / norm(A \ h1);

% Compute rotation
r1 = A \ (lambda * h1);
r2 = A \ (lambda * h2);
r3 = cross(r1, r2);
R = [r1'; r2'; r3'];

% R may not be a true rotation matrix because of noise in the data.
% Find the best rotation matrix to approximate R using SVD.
[U, ~, V] = svd(R);
R = V * U';

% Compute translation vector.
T = (A \ (lambda * h3))';