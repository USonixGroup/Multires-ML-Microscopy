function [camMatrix, reprojectionErrors] = estimateCameraMatrixImpl(imagePoints, worldPoints, filename)
% estimateCameraMatrixImpl Function to estimate camera matrix from imagePoints and worldPoints.

% Copyright 2022 The MathWorks, Inc.

%#codegen

validateInputs(imagePoints, worldPoints, filename);
if ~strcmp(class(imagePoints),class(worldPoints))
    imagePointsInternal = double(imagePoints);
    worldPointsInternal = double(worldPoints);
else
    imagePointsInternal = imagePoints;
    worldPointsInternal = worldPoints;
end

X = worldPointsInternal(:, 1);
Y = worldPointsInternal(:, 2);
Z = worldPointsInternal(:, 3);

M = numel(X);
vec_1 = ones(M,1,'like',X);
vec_0 = zeros(M,1,'like',X);

% Normalize image points
[pts, ~, Tinv] = vision.internal.normalizePoints(imagePointsInternal', 2, class(imagePointsInternal));
u = pts(1, :)';
v = pts(2, :)';

A = [X      Y      Z      vec_1  vec_0  vec_0  vec_0  vec_0  -u.*X  -u.*Y  -u.*Z -u;
     vec_0  vec_0  vec_0  vec_0  X      Y      Z      vec_1  -v.*X  -v.*Y  -v.*Z -v];

[V, D] = eig(A'*A, 'vector');
[~, idx] = min(D); 
P = V(:, idx(1));

camMatrix = reshape(P,4,3)';

% Set back the normalization transform
camMatrix = Tinv * camMatrix;

% Pick a sign
if camMatrix(end) < 0
    camMatrix = -camMatrix;
end

if nargout > 1
    p = camMatrix * [worldPointsInternal, vec_1]';
    x = p(1, :)./p(3, :);
    y = p(2, :)./p(3, :);
    reprojectionErrors = sqrt((imagePointsInternal(:,1)-x').^2+(imagePointsInternal(:,2)-y').^2);
end

end

%--------------------------------------------------------------------------
function validateInputs(imagePoints, worldPoints, filename)
checkImagePoints(imagePoints, filename);
checkWorldPoints(worldPoints, filename);

if size(imagePoints, 1) ~= size(worldPoints, 1)
    error(message('vision:calibrate:numberOfPointsMustMatch'));
end

% Check coplanar case
N = size(worldPoints, 1);
% Compute the centroid
xMean = sum(worldPoints, 1)/N;
% Compute the offset to the centroid
y = worldPoints - repmat(xMean, N, 1);
% Compute the covariance
P =  y' * y;
% Choose the eigenvector with the smallest eigenvalue
[V, D] = eig(P, 'vector');

if all(~real(D))
    error(message('vision:calibrate:coplanarPoints'));
end

[~, idx] = min(D); 
nv = V(:, idx);
        
if all(y * nv < sqrt(eps(class(worldPoints))))
    error(message('vision:calibrate:coplanarPoints'));
end
end

%--------------------------------------------------------------------------
function checkImagePoints(imagePoints, filename)
validateattributes(imagePoints, {'single','double'}, ...
    {'real', 'nonempty', 'finite', 'nonsparse', '2d', 'ncols', 2}, ...
    filename, 'imagePoints');

minNumPoints = 6;
if size(imagePoints, 1) < minNumPoints
    error(message('vision:calibrate:minNumPoints', minNumPoints));
end
end

%--------------------------------------------------------------------------
function checkWorldPoints(worldPoints, filename)
validateattributes(worldPoints, {'single','double'}, ...
    {'real', 'nonempty', 'finite', 'nonsparse', '2d', 'ncols', 3}, ...
    filename, 'worldPoints');
end