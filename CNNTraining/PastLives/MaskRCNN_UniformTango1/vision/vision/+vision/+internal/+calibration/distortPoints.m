function distortedPoints = distortPoints(points, K, radialDistortion, tangentialDistortion)
%

%   Copyright 2014-2024 The MathWorks, Inc.

%#codegen

% unpack the pre-multiply intrinisc matrix
cx = K(1, 3);
cy = K(2, 3);
fx = K(1, 1);
fy = K(2, 2);
skew = K(1, 2);

% center the points
center = [cx, cy];
centeredPoints = bsxfun(@minus, points, center);

% normalize the points
yNorm = centeredPoints(:, 2, :) ./ fy;
xNorm = (centeredPoints(:, 1, :) - skew * yNorm) ./ fx; % Distortion must be computed without skew

% compute radial distortion
r2 = xNorm .^ 2 + yNorm .^ 2;
r4 = r2 .* r2;
r6 = r2 .* r4;

k = zeros(1, 6, 'like', radialDistortion);
numCoeffs = numel(radialDistortion);
k(1:numCoeffs) = radialDistortion(1:numCoeffs);

% branching to avoid unnecessary floating point errors
if numCoeffs <= 3
    alpha = 1 + k(1) * r2 + k(2) * r4 + k(3) * r6;
else
    alpha = (1 + k(1) * r2 + k(2) * r4 + k(3) * r6) ./ (1 + r2 * k(4) + r4 * k(5) + r6 * k(6));
end

% compute tangential distortion
p = tangentialDistortion;
xyProduct = xNorm .* yNorm;
dxTangential = 2 * p(1) * xyProduct + p(2) * (r2 + 2 * xNorm .^ 2);
dyTangential = p(1) * (r2 + 2 * yNorm .^ 2) + 2 * p(2) * xyProduct;

% apply the distortion to the points
normalizedPoints = [xNorm, yNorm];
distortedNormalizedPoints = normalizedPoints .* [alpha, alpha] + ...
    [dxTangential, dyTangential];

% convert back to pixels
distortedPointsX = distortedNormalizedPoints(:, 1, :) * fx + cx + ...
    skew * distortedNormalizedPoints(:, 2, :); % Add skew effect back
distortedPointsY = distortedNormalizedPoints(:, 2, :) * fy + cy;


distortedPoints = [distortedPointsX, distortedPointsY];
