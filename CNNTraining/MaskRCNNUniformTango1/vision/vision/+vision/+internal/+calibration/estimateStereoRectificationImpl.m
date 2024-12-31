function [tform1, tform2] = estimateStereoRectificationImpl(f, inlier_points1, inlier_points2, imageSize, filename, returnProjectiveTransform)
% estimateStereoRectificationImpl Function to estimate projective
% transformations for rectifying stereo images.

% Copyright 2022 The MathWorks, Inc.

%#codegen

%--------------------------------------------------------------------------
% Check the inputs.
%--------------------------------------------------------------------------
[points1, points2] = checkInputs(f, inlier_points1, inlier_points2, ...
    imageSize, filename);

%--------------------------------------------------------------------------
% Compute the projective transformations.
%--------------------------------------------------------------------------
[tform1, tform2] = cvalgEstimateStereoRectification(f, points1, points2, ...
    imageSize, returnProjectiveTransform);
end

%==========================================================================
function [points1, points2] = checkInputs(f, inlier_points1, ...
    inlier_points2, imageSize, filename)
%--------------------------------------------------------------------------
% Check the Fundamental matrix
%--------------------------------------------------------------------------
validateattributes(f, {'double', 'single'}, ...
    {'2d', 'nonsparse', 'nonempty', 'real', 'size', [3,3]},...
    filename, 'F');

%--------------------------------------------------------------------------
% Check the image size
%--------------------------------------------------------------------------
validateattributes(imageSize, {'numeric'}, ...
    {'vector', 'nonsparse', 'nonempty', 'real', 'positive', 'integer'},...
    filename, 'imageSize');

coder.internal.errorIf(length(imageSize) < 2,...
    'vision:estimateUncalibratedRectification:invalidImageSize');

%--------------------------------------------------------------------------
% Check the points
%--------------------------------------------------------------------------
[points1, points2] = ...
    vision.internal.inputValidation.checkAndConvertMatchedPoints(...
    inlier_points1, inlier_points2, filename, 'inlierPoints1','inlierPoints2');
end

function [tform1, tform2] = cvalgEstimateStereoRectification(...
    f, pts1, pts2, imageSize, returnProjectiveTransform)
% Algorithm for computing rectification transformations.
%   This function supports the estimateStereoRectification function.

outputClass = class(f);

%--------------------------------------------------------------------------
% Compute the projective transformation for the second camera.
%--------------------------------------------------------------------------

% Find the epipole
[u, d, v] = svd(f);
epipole = u(:, 3);

% Translate the epipole to put the origin at the center of the image
imageOrigin = cast(0.5, outputClass);
t = [1, 0, -(cast(imageSize(2),outputClass)/2 + imageOrigin); ...
    0, 1, -(cast(imageSize(1),outputClass)/2 + imageOrigin); ...
    0, 0,                                               1];
epipole = t * epipole;

% Move the epipole to the line at y=0 by rotating the image with the
% minimum angle.
% Ensure the homogeneous coordinates have positive sign.
if epipole(3) < 0
    epipole = -epipole;
end

% If the x coordinate is positive, rotate clockwise; otherwise, rotate
% counter-clockwise.
if epipole(1) >= 0
    keepOrientation = -ones(1, outputClass);
else
    keepOrientation = ones(1, outputClass);
end
r = [-epipole(1),  -epipole(2),                                     0;
    epipole(2),  -epipole(1),                                     0;
    0,      0,   sqrt(epipole(2)^2+epipole(1)^2) * keepOrientation];
epipole = r * epipole;

% Move the epipole to the position at infinity.
if epipole(1) ~= 0
    g = [1,            0, 0;
        0,            1, 0;
        -epipole(3)/epipole(1),  0, 1];
else
    g = eye(3, outputClass);
end

% Compute the overall transformation for the second image.
t2 = g * r * t;
t([1,2], 3) = -t([1,2], 3);
t2 = t * t2;
if(t2(end)) ~= 0
    t2 = t2 / t2(end);
end

%--------------------------------------------------------------------------
% Compute the projective transformation for the first camera.
%--------------------------------------------------------------------------

% Compute a (partial) synthetic camera matrix for the second camera.
% This is matrix M, such that F=SM, where S is a skew-symmetric matrix.
% Unfortunately, M cannot be computed using the usual factorization, where
% S = [e]_x and M = [e]_x * F, because then M will be singular. Instead, M
% must be computed using the SVD of F.
% See http://www.robots.ox.ac.uk/~vgg/hzbook/hzbook2/clarification_rectification.pdf
z = cast([ 0, -1, 0;
    1,  0, 0;
    0,  0, 1], outputClass);
d(3,3) = (d(1,1) + d(2,2)) / 2;
m = u * z * d * v';

% Compute the transformation for the first camera so the rows in the first
% camera correspond the rows at the same location in the second camera.
h1r = t2 * m;       % H_0 in Corollary 11.4

% Compute the transformation for the first camera so the points have
% (approximately) the same column locations in the first and second images.
numPoints = size(pts1, 1);
p1 = h1r * [cast(pts1', outputClass); ones(1, numPoints, outputClass)];
p2 = t2  * [cast(pts2', outputClass); ones(1, numPoints, outputClass)];

p2InfinityIdx = abs(p2(3,:)) < eps(outputClass);
p2(3, p2InfinityIdx) = eps(outputClass);
b = p2(1,:) ./ p2(3,:) .* p1(3,:);  % normalize by z's
y = p1' \ b';                       % solve eqn (11.21)
h1c = [y(1), y(2), y(3);            % y(1) --> a, y(2) --> b, y(3) --> c in book
    0,    1,    0;
    0,    0,    1];                 % H_a in eqn (11.20)

% Compute the overall transformation for the first camera.
t1 = h1c * h1r;
if(t1(end)) ~= 0
    t1 = t1 / t1(end);
end

if returnProjectiveTransform
    tform1 = projtform2d(t1);
    tform2 = projtform2d(t2);
else
    tform1 = t1';
    tform2 = t2';
end

%--------------------------------------------------------------------------
% In MATLAB execution, if either epipole is inside I2, issue warning.
%--------------------------------------------------------------------------
if isempty(coder.target)
    %MATLAB execution
    if isPointInImage(imageSize, u(:,3)) || isPointInImage(imageSize, v(:, 3))
        epipole1 = v([1,2],3) / v(3,3);
        epipole2 = u([1,2],3) / u(3,3);
        
        coder.internal.warning(...
            'vision:estimateUncalibratedRectification:epipoleInImage', ...
            num2str(epipole1(1)), num2str(epipole1(2)), ...
            num2str(epipole2(1)), num2str(epipole2(2)), ...
            imageSize(1), imageSize(2));
    end
end
end

%==========================================================================
function isIn = isPointInImage(imageSize, ptHomogeneous)
if ptHomogeneous(3) == 0
    isIn = false;
else
    pt = ptHomogeneous(1:2) / ptHomogeneous(3);
    imageOrigin = [-0.5; -0.5];
    imageEnd = imageSize([2,1]) - 0.5;
    isIn = pt(1, :) >= imageOrigin(1) && pt(1, :) <= imageEnd(1) ...
        && pt(2, :) >= imageOrigin(2) && pt(2, :) <= imageEnd(2);
end

end