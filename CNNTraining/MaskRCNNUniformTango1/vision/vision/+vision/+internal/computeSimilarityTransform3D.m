function T = computeSimilarityTransform3D(points1,points2)
% Fit a similarity transformation between two sets of 3-D points
%   T = computeSimilarityTransform3D(points1,points2) computes a
%   similarity transformation between 2 sets of matched 3-D points.
%   points1 and points2 are M-by-3 arrays of [x,y,z] coordinates. T is a 
%   4-by-4 transformation matrix in the post-multiply convention (matrix
%   times column vector). 
%
%   Note: T must be transposed to be used in most CVT functions.
%
%   References
%   ----------
%   Berthold K. P. Horn, "Closed-form solution of absolute orientation
%   using unit quaternions," J. Opt. Soc. Am. A 4, 629-642 (1987)

%   Copyright 2020-2022 The MathWorks, Inc.

%#codegen

% Find data centroid and remove deviations from centroid.
centroid1 = mean(points1);
centroid2 = mean(points2);

pointsCentroid1 = bsxfun(@minus, points1, centroid1);
pointsCentroid2 = bsxfun(@minus, points2, centroid2);

% Covariance matrix.
M = pointsCentroid2' * pointsCentroid1;

% Compute N: real symmetric 4x4 matrix.
N = zeros(4,'like',points1);
N(1,1) = M(1,1) + M(2,2) + M(3,3);
N(1,2) = M(2,3) - M(3,2);
N(1,3) = M(3,1) - M(1,3);
N(1,4) = M(1,2) - M(2,1);
N(2,2) = M(1,1) - M(2,2) - M(3,3);
N(2,3) = M(1,2) + M(2,1);
N(2,4) = M(3,1) + M(1,3);
N(3,3) = -M(1,1) + M(2,2) - M(3,3);
N(3,4) = M(2,3) + M(3,2);
N(4,4) = -M(1,1) - M(2,2) + M(3,3);

% Since the matrix is symmetric, reflect the values of the upper triangular
% part of the matrix to the lower triangular part.
N = N + triu(N,1)';

% Different results for eig() in codegen.
[V,D] = eig(N,'vector');

% Get the real part of eigenvalues and eigenvectors for codegen to match
% simulation results.
D = real(D);
[~,selectedEigenvalueIdx] = max(D);
quat = real(V(:,selectedEigenvalueIdx).');

% If codegen provides the negative eigenvector, force the positive angle.
if quat(1) < 0
    quat = -quat;
end

R = vision.internal.quaternion.quaternionToRotation(quat');

% Compute scale as ratio of the root-mean-square deviations from centroids.
% It is important to convert the matrix into a vector first, Otherwise
% norm() function does an SVD approximation while computing norm.
scale = norm(pointsCentroid2(:)) / norm(pointsCentroid1(:));

% Include scaling in the rotation matrix.
R = R' * (scale .* eye(3));

% Find translation.
t = centroid2' - R * centroid1';

T = [R t; 0 0 0 1];

end